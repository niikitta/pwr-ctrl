#include <gpiod.h>

#include <boost/asio.hpp>
#include <gpiod.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/asio/property.hpp>

#include <iostream>

namespace buttons_ctrl
{

boost::asio::io_service io;
std::shared_ptr<sdbusplus::asio::connection> conn;

boost::asio::steady_timer timer(io);

std::shared_ptr<sdbusplus::asio::dbus_interface> hostIface;
std::shared_ptr<sdbusplus::asio::dbus_interface> chassisIface;
std::shared_ptr<sdbusplus::asio::dbus_interface> uidledIface;

static const std::string hostDbusName = "xyz.openbmc_project.State.Host";
static const std::string chassisDbusName = "xyz.openbmc_project.State.Chassis";
static const std::string uidledDbusName =
    "xyz.openbmc_project.LED.GroupManager";

static const std::string propertyServerOn =
    "xyz.openbmc_project.State.Host.HostState.Running";
static const std::string propertyServerOff =
    "xyz.openbmc_project.State.Host.HostState.Off";

static const std::string appName = "buttons_ctrl";

typedef struct _GPIO
{
    const std::string name;
    const int offset;
} GPIO;

const GPIO psPwrOk = {"PS_PWROK", 30};
const GPIO pwrBtn = {"POWER_BUTTON", 24};
const GPIO pwrOut = {"POWER_OUT", 25};
const GPIO uidBtn = {"ID_BUTTON", 60};
const GPIO uidOut = {"UID_LED", 62};

const GPIO anomalyPwrOut = {"ANOMALY_POWER_OUT", 61};

// default off uidled value 1
static int uidState = 1;

int getCurrentTimeMs()
{
    return 100;
}

static gpiod::line requestGpios(const GPIO& gpio);

static int setGpios(int offset, int value)
{
    int ret;
    ret = gpiod_ctxless_set_value_ext("gpiochip0", offset, value, false,
                                      appName.c_str(), NULL, NULL, 0);
    if (ret == -1)
    {
        std::cerr << "Failed to set gpio \n";
        return -1;
    }
    return 0;
}

static int getGpios(int offset)
{
    int ret;
    ret = gpiod_ctxless_get_value_ext("gpiochip0", offset, false,
                                      appName.c_str(), 0);
    if (ret == -1)
    {
        std::cerr << "Failed to get gpio\n";
    }
    return ret;
}

static int powerOff()
{
    sdbusplus::asio::setProperty<std::string>(
        *conn, hostDbusName, "/xyz/openbmc_project/state/host0",
        "xyz.openbmc_project.State.Host", "CurrentHostState",
        propertyServerOff.c_str(), [](const boost::system::error_code&) {});

    int ret;
    std::cout << "goPwrOff\n";
    // anomaly mode
    if ((ret = setGpios(anomalyPwrOut.offset, 0)) == -1)
    {
        std::cerr << "anomaly powerOff failed\n";
        return -1;
    }

    // normal mode
    if ((ret = setGpios(pwrOut.offset, 0)) == -1)
    {
        std::cerr << "normal powerOff failed\n";
        return -1;
    }
    return 0;
}

static int powerOn()
{
    sdbusplus::asio::setProperty<std::string>(
        *conn, hostDbusName, "/xyz/openbmc_project/state/host0",
        "xyz.openbmc_project.State.Host", "CurrentHostState",
        propertyServerOn.c_str(), [](const boost::system::error_code&) {});

    int ret;
    // anomaly mode
    if ((ret = setGpios(anomalyPwrOut.offset, 1)) == -1)
    {
        std::cerr << "anomaly powerOn failed\n";
        return -1;
    }

    // normal mode
    if ((ret = setGpios(pwrOut.offset, 0)) == -1)
    {
        std::cerr << "anomaly powerOn failed\n";
        return -1;
    }
    timer.expires_after(std::chrono::milliseconds(200));
    timer.async_wait([&ret](const boost::system::error_code& e) {
        if ((ret = setGpios(pwrOut.offset, 1)) == -1)
        {
            std::cerr << "anomaly powerOn failed\n";
            return;
        }
    });
    return 0;
}

static void powerEventHandler(bool event)
{
    if (event)
    {
        bool wantState;

        if ((wantState = getGpios(psPwrOk.offset)) == -1)
        {
            std::cerr << "Failed to read psPwrOk GPIO\n";
            return;
        }
        std::cout << "before psprok : \n\n" << wantState << "\n\n";

        // opposite value to want state
        wantState = !wantState;

        wantState ? powerOn() : powerOff();
    }
}

static void uidledEventHandler(bool event)
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

static void waitGpios(boost::asio::posix::stream_descriptor& desc,
                      gpiod::line& line, const GPIO& gpio,
                      void (*handler)(bool))
{
    desc.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [&line, &desc, &gpio, &handler](const boost::system::error_code& e) {
            gpiod::line_event line_event = line.event_read();

            if (gpio.name == "ID_BUTTON")
                uidledEventHandler(line_event.event_type ==
                                   gpiod::line_event::RISING_EDGE);
            else if (gpio.name == "POWER_BUTTON")
                powerEventHandler(line_event.event_type ==
                                  gpiod::line_event::RISING_EDGE);
            // handler(line_event.event_type ==
            // gpiod::line_event::RISING_EDGE);
            waitGpios(desc, line, gpio, handler);
        });
}

static gpiod::line requestGpios(const std::string& gpio)
{
    gpiod::line gpioLine = gpiod::find_line(gpio);
    if (!gpioLine)
    {
        std::cerr << "Failed to find GPIO " << gpio << '\n';
        exit(EXIT_FAILURE);
    }

    try
    {
        gpioLine.request({appName, gpiod::line_request::EVENT_BOTH_EDGES, {}});
    }
    catch (const std::exception& e)
    {
        std::cerr << "Failed to request GPIO " << gpio << e.what() << '\n';
        exit(EXIT_FAILURE);
    }
    return gpioLine;
}

static int requestGpios(const GPIO& gpio, gpiod::line& gpioLine,
                        boost::asio::posix::stream_descriptor& desc,
                        void (*handler)(bool))
{
    gpioLine = requestGpios(gpio.name);

    int gpioFd = gpioLine.event_get_fd();
    desc.assign(gpioFd);

    waitGpios(desc, gpioLine, gpio, handler);
    return 0;
}

} // namespace buttons_ctrl

int main()
{
    using namespace buttons_ctrl;

    conn = std::make_shared<sdbusplus::asio::connection>(io);

    conn->request_name(hostDbusName.c_str());
    conn->request_name(chassisDbusName.c_str());
    conn->request_name(uidledDbusName.c_str());

    gpiod::line pwrBtnLine;
    gpiod::line uidBtnLine;
    boost::asio::posix::stream_descriptor pwrBtnDesc(io);
    boost::asio::posix::stream_descriptor uidBtnDesc(io);

    if (requestGpios(pwrBtn, pwrBtnLine, pwrBtnDesc, powerEventHandler) == -1)
        return -1;
    if (requestGpios(uidBtn, uidBtnLine, uidBtnDesc, uidledEventHandler) == -1)
        return -1;

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
                powerOn();
                sdbusplus::asio::setProperty<std::string>(
                    *conn, hostDbusName, "/xyz/openbmc_project/state/host0",
                    "xyz.openbmc_project.State.Host", "CurrentHostState",
                    propertyServerOn.c_str(),
                    [](const boost::system::error_code&) {});
            }
            else if (
                req ==
                "xyz.openbmc_project.State.Host.Transition.ForceWarmReboot")
            {
                // force reboot
            }
            resp = req;
            return 1;
        });

    hostIface->register_property(
        "CurrentHostState",
        getGpios(psPwrOk.offset) ? propertyServerOn.c_str()
                                 : propertyServerOff.c_str(),
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
                powerOff();
                sdbusplus::asio::setProperty<std::string>(
                    *conn, hostDbusName, "/xyz/openbmc_project/state/host0",
                    "xyz.openbmc_project.State.Host", "CurrentHostState",
                    propertyServerOff.c_str(),
                    [](const boost::system::error_code&) {});
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
                                           setGpios(uidOut.offset, 0);
                                           uidState = 1;
                                       }
                                       else if (!req)
                                       {
                                           setGpios(uidOut.offset, 1);
                                           uidState = 0;
                                       }
                                       resp = req;
                                       return 1;
                                   });

    uidledIface->initialize();

    io.run();
    return 0;
}