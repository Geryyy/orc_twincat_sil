#pragma once

enum class EventType {
	NONE = 0,
	WARNING,
	ERROR
};

inline const char* toString(EventType error) {
	switch (error) {
	case EventType::NONE: return "NONE";
	case EventType::WARNING: return "WARNING";
	case EventType::ERROR: return "ERROR";
	default: return "(unknown)";
	}
}
