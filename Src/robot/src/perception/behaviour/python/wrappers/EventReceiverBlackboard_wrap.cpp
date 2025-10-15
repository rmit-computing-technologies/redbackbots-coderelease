#include "utils/EventDefinitions.hpp"

class_<EventReceiverBlackboard, boost::noncopyable>("EventReceiverBlackboard")
    .def("getEventTimeSinceReceived", &EventReceiverBlackboard::getEventTimeSinceReceived)

    /**
     * Calls the getPyEventData method to retrieve event data for a specific player and event.
     *
     * @param playerNum The number identifying the player.
     * @param eventName The name of the event to retrieve data for.
     * @return A PyObject pointer containing the event data.
     */
    .def("getEventData", +[](EventReceiverBlackboard& self, int playerNum, const std::string &eventName) -> object {
        PyObject* result = self.getPyEventData(playerNum, eventName);
        if (result) {
            return object(handle<>(borrowed(result)));
        }
        return object();
    })

    /**
     * Retrieves a list of the latest event data for a given event name, indexed by player number.
     *
     * @param eventName The name of the event to retrieve data for.
     * @return A list of PyObject pointers containing the event data.
     */
    .def("getEventData", +[](EventReceiverBlackboard& self, const std::string &eventName) {
        // Store the vector of PyObject* returned by the blackboard
        auto pyObjects = self.getPyEventData(eventName);
        list pyList;
        for (auto *obj : pyObjects) {
            if (obj) {
                // 'obj' is already a Python object, so we can append it directly
                pyList.append(object(handle<>(borrowed(obj))));
            } else {
                pyList.append(object());
            }
        }
        return pyList;
    })

    /**
     * Retrieves the event data for a specific player.
     *
     * @param playerNum The number identifying the player.
     * @return A dictionary containing the player's event data.
     */
    .def("getPlayerEvents", +[](const EventReceiverBlackboard& self, int playerNum) {
        dict playerDict;
        PlayerEventData data = self.getPlayerEvents(playerNum);
        if (data.isValid()) {
            playerDict["playerNumber"] = data.playerNumber;
            list eventsList;
            for (size_t i = 0; i < EVENTS_ARRAY_SIZE; ++i) {
                if (data.events[i]) {
                    dict eventDict;
                    eventDict["name"] = data.events[i]->eventName;
                    PyObject* pyObj = data.events[i]->getPythonData();
                    if (pyObj) {
                        eventDict["data"] = object(handle<>(borrowed(pyObj)));
                    } else {
                        eventDict["data"] = object();
                    }
                    eventDict["hasReceiveTime"] = data.hasReceiveTime[i];
                    eventDict["receiveTime"] = data.receiveTimes[i].time_since_epoch().count();
                    eventsList.append(eventDict);
                }
            }
            playerDict["events"] = eventsList;
        }
        return playerDict;
    })

    /**
     * Retrieves the event data for all players.
     *
     * @return A list of dictionaries, each containing the event data for a player.
     */
    .def("getEvents", +[](const EventReceiverBlackboard& self) {
        list eventsList;
        for (int playerNum = 1; playerNum < ROBOTS_PER_TEAM + 1; ++playerNum) {
            PlayerEventData data = self.getPlayerEvents(playerNum);
            if (data.isValid()) {
                dict playerDict;
                playerDict["playerNumber"] = data.playerNumber;
                list events = list();
                for (size_t i = 0; i < EVENTS_ARRAY_SIZE; ++i) {
                    if (data.events[i]) {
                        dict eventDict;
                        eventDict["name"] = data.events[i]->eventName;
                        PyObject* pyObj = data.events[i]->getPythonData();
                        if (pyObj) {
                            eventDict["data"] = object(handle<>(borrowed(pyObj)));
                        } else {
                            eventDict["data"] = object();
                        }
                        eventDict["hasReceiveTime"] = data.hasReceiveTime[i];
                        eventDict["receiveTime"] = data.receiveTimes[i].time_since_epoch().count();
                        events.append(eventDict);
                    }
                }
                playerDict["events"] = events;
                eventsList.append(playerDict);
            }
        }
        return eventsList;
    })

    .def("getEventReceiveTime", &EventReceiverBlackboard::getEventReceiveTime)
    ;
