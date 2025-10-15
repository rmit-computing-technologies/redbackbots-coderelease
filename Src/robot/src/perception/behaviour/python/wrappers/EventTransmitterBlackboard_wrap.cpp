#include "blackboard/modules/EventTransmitterBlackboard.hpp"
#include <boost/python.hpp>

class_<EventTransmitterBlackboard, boost::noncopyable>("EventTransmitterBlackboard")
    /**
     * Retrieves the valid event names.
     *
     * @param self A reference to the current object instance.
     * @return A list of valid event names.
     */
    .def("getValidEventNames", +[](const EventTransmitterBlackboard& self) -> list {
        auto names = self.getValidEventNames();
        list pyList;
        for (const auto& name : names) {
            pyList.append(name);
        }
        return pyList;
    })

    /**
     * Updates the send time for a specific event.
     *
     * @param self A reference to the current object instance.
     * @param eventName The name of the event.
     * @param secondsUntilSend The time in seconds until the event is sent.
     */
    .def("updateEventSendTime", &EventTransmitterBlackboard::updateEventSendTime)

    /**
     * Sets the data for a specific event.
     *
     * @param self A reference to the current object instance.
     * @param eventName The name of the event.
     * @param pyObj The Python object containing the event data.
     * @param secondsUntilSend The time in seconds until the event is sent.
     */
    .def("raiseEvent", +[](EventTransmitterBlackboard& self, const std::string &eventName, object pyObj, float secondsUntilSend) {
        try {
            PyObject* rawPtr = pyObj.ptr();
            if (!rawPtr) {
                throw std::runtime_error("Invalid Python object");
            }
            self.raiseEvent(eventName, rawPtr, secondsUntilSend);
        } catch (const std::exception &e) {
            PyErr_SetString(PyExc_RuntimeError, e.what());
            throw_error_already_set();
        }
    })

    .def("getEventData", +[](EventTransmitterBlackboard& self, const std::string &eventName) -> object {
        PyObject* result = self.getPyEventData(eventName);
        if (result) {
            return object(handle<>(borrowed(result)));
        }
        return object();
    })

    /**
     * Retrieves the events.
     *
     * @param self A reference to the current object instance.
     * @return A list of dictionaries containing event details.
     */
    .def("getEvents", +[](const EventTransmitterBlackboard& self) {
        list eventsArray;
        for (size_t i = 0; i < EVENTS_ARRAY_SIZE; ++i) {
            if (self.events[i]) {
                dict eventDict;
                eventDict["name"] = self.events[i]->eventName;
                PyObject* pyObj = self.events[i]->getPythonData();
                if (pyObj) {
                    eventDict["data"] = object(handle<>(borrowed(pyObj)));
                } else {
                    eventDict["data"] = object();
                }
                eventDict["raised"] = self.eventRaisedSet[i];
                eventDict["raisedTime"] = self.eventRaisedTimes[i].time_since_epoch().count();
                eventDict["timeToSend"] = self.eventTimeToSend[i];
                eventDict["sent"] = self.eventSent[i];
                eventDict["sentTime"] = self.eventSentTimes[i].time_since_epoch().count();
                eventsArray.append(eventDict);
            }
        }
        return eventsArray;
    })

    /**
     * Checks if a specific event is raised.
     *
     * @param self A reference to the current object instance.
     * @param eventName The name of the event.
     * @return True if the event is raised, false otherwise.
     */
    .def("isEventRaised", +[](EventTransmitterBlackboard& self, const std::string &eventName) {
        return self.isEventRaised(eventName);
    })

    /**
     * Retrieves the time to send for a specific event.
     *
     * @param self A reference to the current object instance.
     * @param eventName The name of the event.
     * @return The float time to send the event in seconds.
     */
    .def("getEventTimeToSend", &EventTransmitterBlackboard::getEventTimeToSend)

    /**
     * Retrieves the raised time for a specific event.
     *
     * @param self A reference to the current object instance.
     * @param eventName The name of the event.
     * @return The raised time of the event from system_clock.
     */
    .def("getEventRaisedTime", &EventTransmitterBlackboard::getEventRaisedTime)
    ;
