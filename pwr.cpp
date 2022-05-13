#include <gpiod.h>

#include <boost/asio.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/asio/property.hpp>

#include <iostream>

#ifdef ANOMALY_WITH_OFFSET
#define POWER_OUT_DEF 61
#define PS_PWROK_DEF 31
#endif

#ifdef NORMAL_GPIO_STATE
#define POWER_OUT_DEF 25
#define PS_PWROK_DEF 31
#endif

namespace pwr_ctrl
{

boost::asio::io_service io;
std::shared_ptr<sdbusplus::asio::connection> conn;

std::shared_ptr<sdbusplus::asio::dbus_interface> hostIface;
std::shared_ptr<sdbusplus::asio::dbus_interface> chassisIface;

boost::asio::steady_timer timer(io);

static const std::string hostDbusName = "xyz.openbmc_project.State.Host";
static const std::string chassisDbusName = "xyz.openbmc_project.State.Chassis";

static std::string hostStateForCheck =
    "xyz.openbmc_project.State.Host.HostState.";

static std::string currentState = "";

// GPIO line offsets
static const unsigned int PS_PWROK = PS_PWROK_DEF;
static const unsigned int POWER_OUT = POWER_OUT_DEF;

typedef enum
{
    nowOn,
    nowOff,
} State;
State state;

typedef enum
{
    goToOn,
    goToForceOff,
    goToForceReboot,
} Event;

typedef struct
{
    const char* device;
    unsigned int offsets;
    int values;
    unsigned int num_lines;
    bool active_low;
    const char* consumer;
    gpiod_ctxless_set_value_cb cb;
    void* data;
    int flags;
} PwrConfig;

static uint64_t getCurrentTimeMs()
{
    struct timespec time = {};

    if (clock_gettime(CLOCK_REALTIME, &time) < 0)
    {
        return 0;
    }
    uint64_t currentTimeMs = static_cast<uint64_t>(time.tv_sec) * 1000;
    currentTimeMs += static_cast<uint64_t>(time.tv_nsec) / 1000 / 1000;

    return currentTimeMs;
}

static int setGPIOValue(PwrConfig& pwrConfig, const int& value)
{
    int rv;

    rv = gpiod_ctxless_set_value_ext(
        pwrConfig.device, pwrConfig.offsets, value, pwrConfig.active_low,
        pwrConfig.consumer, pwrConfig.cb, pwrConfig.data, pwrConfig.flags);

    if (rv < 0)
    {
        std::cerr << "Error setting GPIO value for GPIO line "
                  << pwrConfig.offsets << std::endl;
    }
    return 0;
}

static int getGPIOValue(PwrConfig& pwrConfig)
{
    int value, rv;

    rv = gpiod_ctxless_get_value_multiple_ext(
        pwrConfig.device, &pwrConfig.offsets, &value, pwrConfig.num_lines,
        pwrConfig.active_low, pwrConfig.consumer, pwrConfig.flags);

    if (rv < 0)
    {
        std::cerr << "Error reading GPIO value for GPIO line "
                  << pwrConfig.offsets << std::endl;
    }
    return value;
}

static State getPowerState(PwrConfig& psPwrCfg)
{
    int value;
    value = getGPIOValue(psPwrCfg);
    if (value == 1)
        return nowOn;
    else if (value == 0)
        return nowOff;
}

static void dbusMetaInfoSet()
{
    std::string stateConcat;
    if (state == nowOn)
    {
        stateConcat = "Running";
    }
    else if (state == nowOff)
    {
        stateConcat = "Off";
    }
    sdbusplus::asio::setProperty<std::string>(
        *conn, "xyz.openbmc_project.State.Host",
        "/xyz/openbmc_project/state/host0", "xyz.openbmc_project.State.Host",
        "CurrentHostState",
        "xyz.openbmc_project.State.Host.HostState." + stateConcat,
        [](const boost::system::error_code&) {});

    sdbusplus::asio::setProperty<int>(
        *conn, chassisDbusName, "/xyz/openbmc_project/state/chassis0",
        "xyz.openbmc_project.State.Chassis", "LastStateChangeTime",
        getCurrentTimeMs(), [](const boost::system::error_code&) {});
}

static void powerOn(PwrConfig& pwrOutCfg)
{
    timer.expires_after(boost::asio::chrono::milliseconds(200));
    setGPIOValue(pwrOutCfg, 0);
    timer.async_wait([&pwrOutCfg](const boost::system::error_code ec) {
        if (ec)
        {
            std::cerr << "async_wait failed: " << ec.message() << std::endl;
            return;
        }
        setGPIOValue(pwrOutCfg, 1);
    });
}

// after force power off front panel button unavailable
static void forcePowerOff(PwrConfig& pwrOutCfg)
{
    setGPIOValue(pwrOutCfg, 0);
}

static void forceReboot(PwrConfig& pwrOutCfg)
{
    forcePowerOff(pwrOutCfg);
    powerOn(pwrOutCfg);
}

/*
 *   I have anomly offset in GPIOD group(power management).
 *   This handler describe power management with this normal mode and
 *   anomaly mode.
 */
static void handlerForGPIOOut(Event event, PwrConfig& pwrOutCfg)
{
    if (event == goToOn)
    {
        std::clog << "Go to power on...\n";
#ifdef NORMAL_GPIO_STATE
        powerOn(pwrOutCfg);
#endif
#ifdef ANOMALY_WITH_OFFSET
        powerOn(pwrOutCfg);
#endif
    }
    else if (event == goToForceOff)
    {
        std::clog << "Go to force power off...\n";
#ifdef NORMAL_GPIO_STATE
        forcePowerOff(pwrOutCfg);
#endif
#ifdef ANOMALY_WITH_OFFSET
        forcePowerOff(pwrOutCfg);
#endif
    }
    else if (event == goToForceReboot)
    {
        std::clog << "Go to force reboot...\n";
#ifdef NORMAL_GPIO_STATE
        forceReboot(pwrOutCfg);
#endif
#ifdef ANOMALY_WITH_OFFSET
        forceReboot(pwrOutCfg);
#endif
    }
}

static void initPwrCfg(PwrConfig& pwrConfig, const unsigned int& gpioOffset)
{
    const int defalut_value = 0;
    pwrConfig = {"gpiochip0", gpioOffset, defalut_value, 1, false,
                 "pwr_ctrl",  NULL,       NULL,          0};
}

} // namespace pwr_ctrl

int main(int argc, char** argv)
{
    using namespace pwr_ctrl;

    conn = std::make_shared<sdbusplus::asio::connection>(io);

    conn->request_name(hostDbusName.c_str());
    conn->request_name(chassisDbusName.c_str());

    PwrConfig pwrOutCfg;
    PwrConfig psPwrOkCfg;
    initPwrCfg(pwrOutCfg, POWER_OUT);
    initPwrCfg(psPwrOkCfg, PS_PWROK);

    state = getPowerState(psPwrOkCfg);
    if (state == nowOn)
    {
        currentState = "xyz.openbmc_project.State.Host.HostState.Running";
    }
    else if (state == nowOff)
    {
        currentState = "xyz.openbmc_project.State.Host.HostState.Off";
    }

    sdbusplus::asio::object_server hostServer =
        sdbusplus::asio::object_server(conn);
    hostIface = hostServer.add_interface("/xyz/openbmc_project/state/host0",
                                         "xyz.openbmc_project.State.Host");
    hostIface->register_property(
        "RequestedHostTransition",
        std::string("xyz.openbmc_project.State.Host.Transition.Off"),
        [&pwrOutCfg](const std::string& req, std::string& resp) {
            if (req == "xyz.openbmc_project.State.Host.Transition.On")
            {
                state = nowOn;
                dbusMetaInfoSet();
                handlerForGPIOOut(goToOn, pwrOutCfg);
            }
            else if (
                req ==
                "xyz.openbmc_project.State.Host.Transition.ForceWarmReboot")
            {
                dbusMetaInfoSet();
                handlerForGPIOOut(goToForceReboot, pwrOutCfg);
            }
            resp = req;
            return 1;
        });
    hostIface->register_property(
        "CurrentHostState", currentState,
        sdbusplus::asio::PropertyPermission::readWrite);
    hostIface->initialize();

    sdbusplus::asio::object_server chassisServer =
        sdbusplus::asio::object_server(conn);
    chassisIface =
        chassisServer.add_interface("/xyz/openbmc_project/state/chassis0",
                                    "xyz.openbmc_project.State.Chassis");
    chassisIface->register_property(
        "RequestedPowerTransition",
        std::string("xyz.openbmc_project.State.Chassis.Transition.Off"),
        [&pwrOutCfg](const std::string& req, std::string resp) {
            if (req == "xyz.openbmc_project.State.Chassis.Transition.Off")
            {
                state = nowOff;
                dbusMetaInfoSet();
                handlerForGPIOOut(goToForceOff, pwrOutCfg);
            }
            resp = req;
            return 1;
        });
    chassisIface->register_property(
        "CurrentPowerState",
        std::string("xyz.openbmc_project.State.Chassis.PowerState.On"),
        sdbusplus::asio::PropertyPermission::readWrite);
    chassisIface->register_property("LastStateChangeTime", getCurrentTimeMs());
    chassisIface->initialize();

    io.run();

    return 0;
}