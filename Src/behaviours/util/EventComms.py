from util import log
from util.MathUtil import nano_to_micro, micro_to_seconds

blackboard = None
valid_event_names = []

# Internal caches of the latest receiver and transmitter data.
_receiver_events = []
_transmitter_events = []

# For quick lookups keyed by player -> { event_name: event_dict (with max receiveTime) }.
# For example, _events_by_player_by_name[1]["BALL_SCORE_UPDATE"] would return the most recent
# BALL_SCORE_UPDATE event received from player 1.
_events_by_player_by_name = {}

def _check_valid_event_name(event_name):
    """
    Checks if the given event name is valid, and raises a ValueError if not.
    """
    if event_name not in valid_event_names:
        raise ValueError(f"Invalid event name: {event_name}")

def update_event_comms(new_blackboard):
    """
    Updates the EventComms.py global variables.

    Callable via `EventComms.update_event_comms(blackboard)`.

    :param new_blackboard: What to update the globals in this module with.
    :return: None
    """
    global blackboard, valid_event_names
    # log.debug("Updating EventComms with new blackboard.")
    blackboard = new_blackboard

    valid_event_names = blackboard.eventTransmitter.getValidEventNames()
    # log.debug(f"Valid event names: {valid_event_names}")
    sync_blackboard_data()
    
    # log.info("Raising python test event")
    # raise_event("BALL_SCORE_UPDATE", 321, 5.0)


def raise_event(event_name: str, event_data=None, time_to_send: float=0.0):
    """
    Raises an event with the given name and data.

    Callable via `EventComms.raise_event(event_name, event_data (Optional), time_to_send (Optional))`.

    :param event_name: The name of the event to raise.
    :param event_data: The data to send with the event.
    :param time_to_send: The maximum time to send the event in seconds.
    :return: None
    """
    _check_valid_event_name(event_name)
    log.debug(f"Raising event: {event_name}, data: {event_data}, time_to_send: {time_to_send}")
    blackboard.eventTransmitter.raiseEvent(event_name, event_data, time_to_send)


def sync_blackboard_data():
    """
    Synchronizes blackboard event data by retrieving receiver and transmitter events,
    logging them, and building a lookup dictionary of the latest events for each player
    organized by event name.

    This function performs the following steps:
    - Retrieves events from the blackboard's event receiver and transmitter.
    - Logs all received and transmitted events.
    - Constructs a nested dictionary mapping each player's number to their events,
      ensuring that only the most recent event (based on receiveTime) is stored for each event name.
    """
    global _receiver_events, _transmitter_events, _events_by_player_by_name
    # log.debug("Syncing blackboard data.")

    _receiver_events = blackboard.eventReceiver.getEvents()
    # log.info("Receiver Events:")
    # for event in _receiver_events:
    #     log.info(event)
    #     print("")

    _transmitter_events = blackboard.eventTransmitter.getEvents()
    # log.info("Transmitter Events:")
    # for event in _transmitter_events:
    #     log.info(event)
    #     print("")

    # Build a lookup of the latest event by (player -> event_name).
    _events_by_player_by_name = {}
    for entry in _receiver_events:
        player_num = entry["playerNumber"]
        if player_num not in _events_by_player_by_name:
            _events_by_player_by_name[player_num] = {}
        for event in entry["events"]:
            name = event["name"]
            # If multiple events share the same name, keep the one with the largest receiveTime.
            if name not in _events_by_player_by_name[player_num]:
                _events_by_player_by_name[player_num][name] = event
            else:
                if event["receiveTime"] > _events_by_player_by_name[player_num][name]["receiveTime"]:
                    _events_by_player_by_name[player_num][name] = event

# Receiver functions

def last_received_by_player(player_number: int) -> dict:
    """
    Returns the events received in the most recent packet by the given player number.

    Callable via `EventComms.last_received(player_number)`.

    :param player_number: The player number to get the last events received for.
    :return: A dictionary of the last received event data for a given player number.
             In the form: { event_name: event_data }
    """
    events_map = _events_by_player_by_name.get(player_number, {})
    if not events_map:
        return {}

    # Find the maximum receiveTime (Time of the most recent packet)
    max_receive_time = max(event["receiveTime"] for event in events_map.values())

    # Return events that have the most recent receiveTime
    return {name: event["data"] for name, event in events_map.items() if event["receiveTime"] == max_receive_time}

def last_received_by_name(event_name: str):
    """
    Returns a tuple containing the event data and the player number from whom the given event was last received.
    Callable via `EventComms.last_received(player_number)`.
    
    :param event_name: The name of the event to get the last received event for.
    :return: A tuple containing:
             - The event data from the player.
             - The player number from whom the event was last received.
             Returns (None, None) if no such event is found.
    """
    _check_valid_event_name(event_name)
    found_event = None
    found_player = None
    for player_num, event_map in _events_by_player_by_name.items():
        if event_name in event_map:
            event = event_map[event_name]
            if not found_event or event["receiveTime"] > found_event["receiveTime"]:
                found_event = event
                found_player = player_num
    return (found_event["data"], found_player) if found_event else (None, None)


def last_received_player_by_name(event_name: str) -> int:
    """
    Returns the player number who we last received the given event from.

    Callable via `EventComms.last_received_player(event_name)`.

    :param event_name: The name of the event to get the last received player number for.
    :return: The player number who we last received the given event from.
    """
    _check_valid_event_name(event_name)
    found_event = None
    found_player = None
    for player_num, event_map in _events_by_player_by_name.items():
        if event_name in event_map:
            event = event_map[event_name]
            if not found_event or event["receiveTime"] > found_event["receiveTime"]:
                found_event = event
                found_player = player_num
    return found_player if found_player else -1


def events_by_player(player_number: int) -> list:
    """
    Returns the last events received by the given player number.

    Callable via `EventComms.last_received_events(player_number)`.

    :param player_number: The player number to get the last events received for.
    :return: A list of the last received event names for a given player number.
    """
    return list(_events_by_player_by_name.get(player_number, {}).keys())


def time_since_received_by_player(player_number: int) -> int:
    """
    Returns the time since the last event was received by the given player number.

    Callable via `EventComms.time_since_received(player_number)`.

    :param player_number: The player number to get the time since the last event received for.
    :return: The time since the last event was received by the given player number, in microseconds.
    """
    events_map = _events_by_player_by_name.get(player_number, {})
    if not events_map:
        return None
    last_time = max(event["receiveTime"] for event in events_map.values())
    
    # Return the microseconds since the last event was received.
    return int(blackboard.vision.timestamp) - nano_to_micro(last_time) if last_time else None


def time_since_received_by_player_event(player_number: int, event_name: str):
    """
    Returns the time since the last event was received by the given player number.

    Callable via `EventComms.time_since_received(player_number, event_name)`.

    :param player_number: The player number to get the time since the last event received for.
    :param event_name: The name of the event to get the time since the last received event for.
    :return: The time since the last event was received by the given player number.
    """
    _check_valid_event_name(event_name)
    events_map = _events_by_player_by_name.get(player_number, {})
    event = events_map.get(event_name)
    if not event:
        return None
    return int(blackboard.vision.timestamp) - nano_to_micro(event["receiveTime"]) if event["receiveTime"] else None


def time_since_received_by_name(event_name: str):
    """
    Returns the time since the last event was received for the given event name.
    From any player number.

    Callable via `EventComms.time_since_received(event_name)`.

    :param event_name: The name of the event to get the time since the last received event for.
    :return: The time since the last event was received for the given event name.
    """
    _check_valid_event_name(event_name)
    found_event = None
    for event_map in _events_by_player_by_name.values():
        if event_name in event_map:
            event = event_map[event_name]
            if not found_event or event["receiveTime"] > found_event["receiveTime"]:
                found_event = event
    return (
        int(blackboard.vision.timestamp) - nano_to_micro(found_event["receiveTime"])
        if found_event and found_event["receiveTime"]
        else None
    )


def seconds_since_received_by_player(player_number: int) -> float:
    """
    Returns a float of the time since the last event was received by the given player number.
    """
    result = time_since_received_by_player(player_number)
    return micro_to_seconds(result) if result is not None else None


def seconds_since_received_by_player_event(player_number: int, event_name: str) -> float:
    """
    Returns a float of the time since the last event was received by the given player number for the specified event.
    """
    result = time_since_received_by_player_event(player_number, event_name)
    return micro_to_seconds(result) if result is not None else None


def seconds_since_received_by_name(event_name: str) -> float:
    """
    Returns a float of the time since the last event was received for the given event name, from any player number.
    """
    result = time_since_received_by_name(event_name)
    return micro_to_seconds(result) if result is not None else None


def received_data_by_player(player_number: int):
    """
    Returns a dictionary of all event data for a given player number.

    Callable via `EventComms.received_data(player_number)`.

    :param player_number: The player number to get the data of the last event received for.
    :return: A dictionary of all event data for a given player number.
             In the form: { event_name: event_data }
    """
    events_map = _events_by_player_by_name.get(player_number, {})
    return {name: event["data"] for name, event in events_map.items()}


def received_data_by_player_event(player_number: int, event_name: str):
    """
    Returns last received data for a given event by the given player number.

    Callable via `EventComms.received_data(player_number, event_name)`.

    :param player_number: The player number to get the data of the event for.
    :param event_name: The name of the event to get the data for.
    :return: The data of the event from the given player number.
    """
    _check_valid_event_name(event_name)
    events_map = _events_by_player_by_name.get(player_number, {})
    event = events_map.get(event_name)
    return event["data"] if event else None


def received_data_by_name(event_name: str) -> dict:
    """
    Returns a dictionary of all event data for a given event name.

    Callable via `EventComms.received_data(event_name)`.

    :param event_name: The name of the event to get the data of.
    :return: A dictionary of all event data for a given event name.
             In the form: { player_number: event_data }
    """
    _check_valid_event_name(event_name)
    data_map = {}
    for player_num, event_map in _events_by_player_by_name.items():
        if event_name in event_map:
            data_map[player_num] = event_map[event_name]["data"]
    return data_map

# Transmitter functions

def _get_transmitter_event(event_name: str) -> dict:
    """
    Retrieves the transmitter event dictionary for the given event name.

    :param event_name: The name of the event to retrieve.
    :return: The event dictionary if found, else None.
    """
    _check_valid_event_name(event_name)
    for event in _transmitter_events:
        if event["name"] == event_name:
            return event
    return None


def sent(event_name: str) -> bool:
    """
    Returns whether the given event has ever been sent by this robot.

    Callable via `EventComms.sent(event_name)`.

    :param event_name: The name of the event to check if it has been sent.
    :return: True if the event has been sent, False otherwise.
    """
    _check_valid_event_name(event_name)
    event = _get_transmitter_event(event_name)
    return event["sent"] if event else False


def sent_since(time_raised: int, event_name: str) -> bool:
    """
    Returns whether an event has been sent since the given time.

    Callable via `EventComms.sent_since(time_raised, event_name)`.

    :param time_raised: The time to check if an event has been sent since.
    :param event_name: The name of the event to check.
    :return: True if the event has been sent since the given time, False otherwise.
    """
    _check_valid_event_name(event_name)
    event = _get_transmitter_event(event_name)
    return event["sentTime"] > time_raised if event and event["sentTime"] else False

def is_raised(event_name: str) -> bool:
    """
    Returns whether the given event has been raised.

    Callable via `EventComms.is_raised(event_name)`.

    :param event_name: The name of the event to check if it has been raised.
    :return: True if the event has been raised, False otherwise.
    """
    _check_valid_event_name(event_name)
    return blackboard.eventTransmitter.isEventRaised(event_name)
