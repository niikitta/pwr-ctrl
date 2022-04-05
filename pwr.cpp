#include <boost/asio.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/asio/property.hpp>

#include <iostream>

namespace pwr_ctrl
{
boost::asio::io_service io;
std::shared_ptr<sdbusplus::asio::connection> conn;

std::shared_ptr<sdbusplus::asio::dbus_interface> hostIface;
std::shared_ptr<sdbusplus::asio::dbus_interface> chassisIface;

static std::string hostDbusName = "xyz.openbmc_project.State.Host";
static std::string chassisDbusName = "xyz.openbmc_project.State.Chassis";

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

void setGPIOOutput(Event event)
{
    if (event == Event::on)
    {
        system("gpioset gpiochip0 25=0");
    }
    else if (event == Event::off)
    {
        system("gpioset gpiochip0 25=1");
    }
}

} // namespace pwr_ctrl

int main()
{
    using namespace pwr_ctrl;
    conn = std::make_shared<sdbusplus::asio::connection>(io);

    conn->request_name(hostDbusName.c_str());
    conn->request_name(chassisDbusName.c_str());

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
                setGPIOOutput(Event::on);
                sdbusplus::asio::setProperty<std::string>(
                    *conn, "xyz.openbmc_project.State.Host",
                    "/xyz/openbmc_project/state/host0",
                    "xyz.openbmc_project.State.Host", "CurrentHostState",
                    "xyz.openbmc_project.State.Host.HostState.Running",
                    [](const boost::system::error_code& ec) {});
            }
            else
            {
                std::cout << "pwr-ctrl: Undefined request to host\n";
            }
            resp = req;
            return 1;
        });
    hostIface->register_property(
        "CurrentHostState",
        std::string("xyz.openbmc_project.State.Host.HostState.Running"),
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
                setGPIOOutput(Event::off);
                sdbusplus::asio::setProperty<std::string>(
                    *conn, hostDbusName, "/xyz/openbmc_project/state/host0",
                    "xyz.openbmc_project.State.Host", "CurrentHostState",
                    "xyz.openbmc_project.State.Host.HostState.Off",
                    [](const boost::system::error_code& ec) {});
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