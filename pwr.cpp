#include <gpiod.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <gpiod.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/asio/property.hpp>

#include <iostream>

#define PS_PWROK_GPIO 30
#define POWER_OUT_GPIO 25
#define UID_OUT_GPIO 62

// we have anomly offset gpio
#define ANOMALY_POWER_OUT_GPIO 61

namespace pwr_ctrl
{
boost::asio::io_service io;
std::shared_ptr<sdbusplus::asio::connection> conn;

std::shared_ptr<sdbusplus::asio::dbus_interface> hostIface;
std::shared_ptr<sdbusplus::asio::dbus_interface> chassisIface;
std::shared_ptr<sdbusplus::asio::dbus_interface> uidledIface;

static const std::string hostDbusName = "xyz.openbmc_project.State.Host";
static const std::string chassisDbusName = "xyz.openbmc_project.State.Chassis";
static const std::string uidledDbusName =
    "xyz.openbmc_project.LED.GroupManager";

static std::string hostStateForCheck =
    "xyz.openbmc_project.State.Host.HostState.";

int serverState;

// default off uidled value 1
static int uidState = 1;

typedef struct
{
    const char* device = "gpiochip0";
    bool aclive_low = false;
    const char* consumer = "pwr-ctrl";
    int flags = 0;
} gpioPsPowerOkConfig;
gpioPsPowerOkConfig psPowerOkConfig;

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

static void setGPIOs(const int& offset, const int& value)
{
    gpiod_ctxless_set_value_ext(
        psPowerOkConfig.device, offset, value, psPowerOkConfig.aclive_low,
        psPowerOkConfig.consumer, NULL, NULL, psPowerOkConfig.flags);
}

static void powerOn(const int& gpioLine)
{
    setGPIOs(gpioLine, 0);
    setGPIOs(gpioLine, 1);
}

static void powerOff(const int& gpioLine)
{
    setGPIOs(gpioLine, 0);
}

static void hardReset(const int& gpioLine)
{
    powerOff(gpioLine);
    powerOn(gpioLine);
}

static void setDbusPwrState(const int& currentState)
{
    static std::string concatState;

    if (currentState)
        concatState = "Running";
    else
        concatState = "Off";
    sdbusplus::asio::setProperty<std::string>(
        *conn, "xyz.openbmc_project.State.Host",
        "/xyz/openbmc_project/state/host0", "xyz.openbmc_project.State.Host",
        "CurrentHostState", hostStateForCheck + concatState,
        [](const boost::system::error_code&) {});
}

static void readPsPwrOk(const boost::system::error_code&,
                        boost::asio::deadline_timer* timer)
{
    serverState = gpiod_ctxless_get_value_ext(
        psPowerOkConfig.device, PS_PWROK_GPIO, psPowerOkConfig.aclive_low,
        psPowerOkConfig.consumer, psPowerOkConfig.flags);
    setDbusPwrState(serverState);

    timer->expires_at(timer->expires_at() + boost::posix_time::seconds(1));
    timer->async_wait(
        boost::bind(readPsPwrOk, boost::asio::placeholders::error, timer));
}

static void pwrBtnHandler(bool event)
{
    if (event)
    {
        // want to off
        if (serverState)
        {
            // nomal power off
            powerOff(POWER_OUT_GPIO);
            // anomaly power off
            powerOff(ANOMALY_POWER_OUT_GPIO);
        }
        else // want to on
        {
            // nomal power on
            powerOn(POWER_OUT_GPIO);
            // anomaly power on
            powerOn(ANOMALY_POWER_OUT_GPIO);
        }
    }
}

static void uidBtnHandler(bool event)
{
    if (event)
    {
        sdbusplus::asio::setProperty<bool>(
            *conn, uidledDbusName,
            "/xyz/openbmc_project/led/groups/enclosure_identify",
            "xyz.openbmc_project.Led.Group", "Asserted",
            static_cast<int>(!uidState),
            [](const boost::system::error_code&) {});
    }
}

static void waitButtons(const std::string& name, gpiod::line& line,
                        boost::asio::posix::stream_descriptor& desc,
                        const std::function<void(bool)>& handler)
{
    desc.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [&name, &line, &desc, &handler](const boost::system::error_code& e) {
            gpiod::line_event event = line.event_read();
            if (name == "POWER_BUTTON")
                pwrBtnHandler(event.event_type ==
                              gpiod::line_event::RISING_EDGE);
            else if (name == "ID_BUTTON")
                uidBtnHandler(event.event_type ==
                              gpiod::line_event::RISING_EDGE);
            // handler(event.event_type == gpiod::line_event::RISING_EDGE);
            waitButtons(name, line, desc, handler);
        });
}

static int requestButtons(const std::string& name, gpiod::line& line,
                          boost::asio::posix::stream_descriptor& desc,
                          const std::function<void(bool)>& handler)
{
    line = gpiod::find_line(name);
    if (!line)
    {
        std::cerr << "Failed to find button " << name << '\n';
        return -1;
    }

    try
    {
        line.request({"pwr-ctrl", gpiod::line_request::EVENT_BOTH_EDGES, {}});
    }
    catch (const std::exception& e)
    {
        std::cerr << "Failed to request gpio " << e.what() << name << '\n';
        return -1;
    }

    int fd = line.event_get_fd();
    desc.assign(fd);
    waitButtons(name, line, desc, handler);

    return 0;
}

}; // namespace pwr_ctrl

int main()
{
    using namespace pwr_ctrl;
    conn = std::make_shared<sdbusplus::asio::connection>(io);

    conn->request_name(hostDbusName.c_str());
    conn->request_name(chassisDbusName.c_str());
    conn->request_name(uidledDbusName.c_str());

    gpiod::line pwrBtnLine;
    gpiod::line uidBtnLine;
    boost::asio::posix::stream_descriptor pwrBtnDesc(io);
    boost::asio::posix::stream_descriptor uidBtnDesc(io);
    const std::string pwrBtnName = "POWER_BUTTON";
    const std::string uidBtnName = "ID_BUTTON";

    if (requestButtons(pwrBtnName, pwrBtnLine, pwrBtnDesc, pwrBtnHandler) == -1)
        return -1;
    if (requestButtons(uidBtnName, uidBtnLine, uidBtnDesc, uidBtnHandler) == -1)
        return -1;

    boost::asio::deadline_timer timer(io, boost::posix_time::seconds(1));

    timer.async_wait(
        boost::bind(readPsPwrOk, boost::asio::placeholders::error, &timer));

    sdbusplus::asio::object_server hostServer =
        sdbusplus::asio::object_server(conn);
    hostIface = hostServer.add_interface("/xyz/openbmc_project/state/host0",
                                         "xyz.openbmc_project.State.Host");
    hostIface->register_property(
        "RequestedHostTransition",
        std::string("xyz.openbmc_project.State.Host.Transition.Off"),
        [](const std::string& req, std::string& resp) {
            if (req == "xyz.openbmc_project.State.Host.Transition.On")
            {
                // nomal power on
                powerOn(POWER_OUT_GPIO);
                // anomaly power on
                powerOn(ANOMALY_POWER_OUT_GPIO);
            }
            else if (
                req ==
                "xyz.openbmc_project.State.Host.Transition.ForceWarmReboot")
            {
                // nomal reset
                hardReset(POWER_OUT_GPIO);
                // anomaly reset
                hardReset(ANOMALY_POWER_OUT_GPIO);
            }
            resp = req;
            return 1;
        });

    hostIface->register_property(
        "CurrentHostState", hostStateForCheck + "Running",
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
        [](const std::string& req, std::string resp) {
            if (req == "xyz.openbmc_project.State.Chassis.Transition.Off")
            {
                // nomal power off
                powerOff(POWER_OUT_GPIO);
                // anomaly power off
                powerOff(ANOMALY_POWER_OUT_GPIO);
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

    sdbusplus::asio::object_server uidledServer =
        sdbusplus::asio::object_server(conn);
    uidledIface = uidledServer.add_interface(
        "/xyz/openbmc_project/led/groups/enclosure_identify",
        "xyz.openbmc_project.Led.Group");
    uidledIface->register_property("Asserted", false,
                                   [](const bool& req, bool& resp) {
                                       if (req)
                                       {
                                           setGPIOs(UID_OUT_GPIO, 0);
                                           uidState = 1;
                                       }
                                       else if (!req)
                                       {
                                           setGPIOs(UID_OUT_GPIO, 1);
                                           uidState = 0;
                                       }
                                       resp = req;
                                       return 1;
                                   });

    uidledIface->initialize();

    io.run();
    return 0;
}