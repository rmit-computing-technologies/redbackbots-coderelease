#pragma once

#include <array>
#include <limits>

/**
 * @enum Criticality
 * Represents the priority tier of an event. Events with higher criticality tiers
 * are sent more aggressively, even if packet usage is high.
 */
enum class Criticality {
    LOW,
    MEDIUM,
    HIGH,
    CRITICAL
};

inline std::ostream& operator<<(std::ostream& os, const Criticality& criticality) {
    switch (criticality) {
        case Criticality::LOW:
            os << "LOW";
            break;
        case Criticality::MEDIUM:
            os << "MEDIUM";
            break;
        case Criticality::HIGH:
            os << "HIGH";
            break;
        case Criticality::CRITICAL:
            os << "CRITICAL";
            break;
        default:
            os << "UNKNOWN";
            break;
    }
    return os;
}

/**
 * @struct CriticalityUsageThreshold
 * Defines a range of estimated total game projected total packet usages [minUsageProjection, maxUsageProjection)
 * at which this event tier is allowed to trigger sending a packet when its time to send reaches 0.
 * The usageProjectionImpact represents the maximum allowed projection of total packet usage
 * for the event to be eligible to send.
 *
 * - If the current projected total packet usage is < usageProjectionImpact, the event can trigger sending.
 * - If the current projected total packet usage is >= usageProjectionImpact, the event will not trigger sending.
 * - For HIGH and CRITICAL criticalities, usageProjectionImpact is set to infinity, allowing them to always send.
 */
struct CriticalityUsageThreshold {
    Criticality level;
    float minUsageProjection;          // "Sent earlier if we predict we will use less than this packet usage"
    float maxUsageProjection;          // "Sent later if we predict we will exceed this packet usage"
    float usageProjectionImpact;       // "We must not project to exceed this, for this event to trigger a packet"
};;

// Each tier's [minUsageProjection, maxUsageProjection) boundaries and usageProjectionImpact
/**
 * @brief Configuration for handling different criticality levels in packet sending.
 *
 * This array defines the behavior of events based on their criticality and the current
 * estimated total game projected total packet usage.
 *
 * @details
 * Each `CriticalityUsageThreshold` includes:
 * - **Criticality Level**: Defines the priority of the event (LOW, MEDIUM, HIGH, CRITICAL).
 * - **minUsageProjection**: The minimum projected total packet usage required for the event to trigger a send.
 * - **maxUsageProjection**: The maximum projected total packet usage beyond which the event is no longer effective.
 * - **maxImpact**: The upper limit for the usage projection impact, set to infinity for always allowed sends.
 *
 * **Event Time Scaling:**
 * - The event time is scaled based on `minUsageProjection`.
 * - If the current projected total packet usage is below `minUsageProjection`, the event's send time is delayed.
 * - If the projected total packet usage exceeds `minUsageProjection`, the event may be sent earlier than initially requested.
 *
 * **Criticality Behaviors:**
 * - **LOW**:
 *   - Effective when projected total packet usage < 0.75.
 *   - Send time is delayed if usage projection >= 0.75.
 * - **MEDIUM**:
 *   - Effective when 0.75 <= projected total packet usage < 1.00.
 *   - Send time is delayed if usage projection >= 1.00.
 * - **HIGH**:
 *   - Effective when 1.00 <= projected total packet usage < 1.10.
 *   - Always sent when time to send reaches 0, regardless of usage projection.
 * - **CRITICAL**:
 *   - No upper bound on projected total packet usage.
 *   - Always prioritized and sent immediately when time to send reaches 0.
 */
/**
 * @brief Configuration for criticality thresholds.
 *
 * Each entry specifies the parameters for a criticality level:
 * - **Criticality level**: The severity of the criticality.
 * - **minUsageProjection**: Sent earlier if projected packet usage is below this value.
 * - **maxUsageProjection**: Sent later if projected packet usage exceeds this value.
 * - **usageProjectionImpact**: Must not be exceeded to trigger a packet for this event.
 */
constexpr std::array<CriticalityUsageThreshold, 4> CriticalityDetails = {{
    // For LOW:
    {
        Criticality::LOW,
        0.0f,   // Never sent earlier
        0.75f,  // Sent later if projected usage exceeds 0.75
        1.00f   // Can trigger a packet if projected usage is below 1.0
    },

    // For MEDIUM:
    {
        Criticality::MEDIUM,
        0.50f,  // Sent earlier if projected usage is below 0.50
        0.90f,  // Sent later if projected usage exceeds 0.90
        std::numeric_limits<float>::infinity()  // Always allowed to send
    },

    // For HIGH:
    {
        Criticality::HIGH,
        0.75f,  // Sent earlier if projected usage is below 0.75  
        1.00f,  // Sent later if projected usage exceeds 1.00
        std::numeric_limits<float>::infinity()  // Always allowed to send
    },

    // For CRITICAL:
    {
        Criticality::CRITICAL,
        1.10f,  // Sent earlier if projected usage is below 1.10
        std::numeric_limits<float>::infinity(), // Never sent later
        std::numeric_limits<float>::infinity()  // Always allowed to send
    }
}};
