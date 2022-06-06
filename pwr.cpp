#include <gpiod.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/asio/property.hpp>

#include <iostream>

#define PS_PWROK_GPIO 30
#define POWER_OUT_GPIO 25

// we have anomly offset gpio
#define ANOMALY_POWER_OUT_GPIO 61

namespace pwr_ctrl
{
boost::asio::io_service io;
std::shared_ptr<sdbusplus::asio::connection> conn;

std::shared_ptr<sdbusplus::asio::dbus_interface> hostIface;
std::shared_ptr<sdbusplus::asio::dbus_interface> chassisIface;

static const std::string hostDbusName = "xyz.openbmc_project.State.Host";
static const std::string chassisDbusName = "xyz.openbmc_project.State.Chassis";

static std::string hostStateForCheck =
    "xyz.openbmc_project.State.Host.HostState.";

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
    int state;

    state = gpiod_ctxless_get_value_ext(
        psPowerOkConfig.device, PS_PWROK_GPIO, psPowerOkConfig.aclive_low,
        psPowerOkConfig.consumer, psPowerOkConfig.flags);

    setDbusPwrState(state);

    timer->expires_at(timer->expires_at() + boost::posix_time::seconds(1));
    timer->async_wait(
        boost::bind(readPsPwrOk, boost::asio::placeholders::error, timer));
}

}; // namespace pwr_ctrl

int main()
{
    using namespace pwr_ctrl;
    conn = std::make_shared<sdbusplus::asio::connection>(io);

    conn->request_name(hostDbusName.c_str());
    conn->request_name(chassisDbusName.c_str());

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

    io.run();
    return 0;
}