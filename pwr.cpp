#include <boost/asio.hpp>
#include <gpiod.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/asio/property.hpp>

#include <iostream>

namespace pwr_ctrl
{
boost::asio::io_service io;
std::shared_ptr<sdbusplus::asio::connection> conn;

std::shared_ptr<sdbusplus::asio::dbus_interface> hostIface;
std::shared_ptr<sdbusplus::asio::dbus_interface> chassisIface;

static const std::string hostDbusName = "xyz.openbmc_project.State.Host";
static const std::string chassisDbusName = "xyz.openbmc_project.State.Chassis";

static std::string currentState = "";

static gpiod::line psPwrOkLine;
static gpiod::line pwrOutLine;

enum class Event
{
    on,
    off,
};

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

static bool requestGPIOEvents(const std::string gpio, gpiod::line& line)
{
    gpiod::line line = gpiod::find_line(gpio.c_str());
    try
    {
        line.request({"pwr-ctrl", gpiod::line_request::EVENT_BOTH_EDGES});
    }
    catch (const std::exception&)
    {
        std::cout << "pwr-ctrl: failed to found gpio line " << gpio << '\n';
        return false;
    }

    return true;
}

static void changeHostState()
{
    if (psPwrOkLine.get_value())
    {
        currentState = "xyz.openbmc_project.State.Host.HostState.Running";
    }
    else
    {
        currentState = "xyz.openbmc_project.State.Host.HostState.Off";
    }
}

static void GPIOPowerOperations(Event event)
{
    if (event == Event::on)
    {
        // pwrOutLine.set_value(0);
        system("gpioset gpiochip0 61=1");
    }
    else if (event == Event::off)
    {
        // pwrOutLine.set_value(1);
        system("gpioset gpiochip0 61=0");
    }
}

} // namespace pwr_ctrl

int main()
{
    using namespace pwr_ctrl;
    conn = std::make_shared<sdbusplus::asio::connection>(io);

    conn->request_name(hostDbusName.c_str());
    conn->request_name(chassisDbusName.c_str());

    if (!requestGPIOEvents("PS_PWROK", psPwrOkLine))
        return -1;
    if (!requestGPIOEvents("POWER_OUT", pwrOutLine))
        return -1;

    changeHostState();

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
                std::cout << "Power On...\n";
                GPIOPowerOperations(Event::on);
                sdbusplus::asio::setProperty<std::string>(
                    *conn, "xyz.openbmc_project.State.Host",
                    "/xyz/openbmc_project/state/host0",
                    "xyz.openbmc_project.State.Host", "CurrentHostState",
                    "xyz.openbmc_project.State.Host.HostState.Running",
                    [](const boost::system::error_code&) {});
                sdbusplus::asio::setProperty<int>(
                    *conn, chassisDbusName,
                    "/xyz/openbmc_project/state/chassis0",
                    "xyz.openbmc_project.State.Chassis", "LastStateChangeTime",
                    getCurrentTimeMs(),
                    [](const boost::system::error_code&) {});
            }
            else
            {
                std::cout << "pwr-ctrl: Undefined request to host\n";
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
        [](const std::string& req, std::string resp) {
            if (req == "xyz.openbmc_project.State.Chassis.Transition.Off")
            {
                std::cout << "Power Off...\n";
                GPIOPowerOperations(Event::off);
                sdbusplus::asio::setProperty<std::string>(
                    *conn, hostDbusName, "/xyz/openbmc_project/state/host0",
                    "xyz.openbmc_project.State.Host", "CurrentHostState",
                    "xyz.openbmc_project.State.Host.HostState.Off",
                    [](const boost::system::error_code&) {});
                sdbusplus::asio::setProperty<int>(
                    *conn, chassisDbusName,
                    "/xyz/openbmc_project/state/chassis0",
                    "xyz.openbmc_project.State.Chassis", "LastStateChangeTime",
                    getCurrentTimeMs(),
                    [](const boost::system::error_code&) {});
            }
            else
            {
                std::cout << "pwr-ctrl: Undefined request to chassis\n";
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
