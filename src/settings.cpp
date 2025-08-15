#include "settings.h"

namespace Settings {
    static Preferences prefs;

    void begin() {
        prefs.begin("pump", false);
    }

    int getInt(const char *key, int defaultValue) {
        return prefs.getInt(key, defaultValue);
    }

    void setInt(const char *key, int value) {
        prefs.putInt(key, value);
    }
}
