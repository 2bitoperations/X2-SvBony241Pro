#pragma once

// Stringify helpers (needed to turn a -D macro integer into a string literal).
#define _SV241PRO_STR(x) #x
#define SV241PRO_STR(x)  _SV241PRO_STR(x)

// Major version — increment manually for breaking/significant changes.
#define PLUGIN_VERSION_MAJOR 1

// BUILD_NUMBER is injected by CI as -DBUILD_NUMBER=<git-commit-count>.
// Local builds omit it; the version string degrades gracefully to just "1".
#ifdef BUILD_NUMBER
#  define PLUGIN_VERSION_STRING SV241PRO_STR(PLUGIN_VERSION_MAJOR) "." SV241PRO_STR(BUILD_NUMBER)
#else
#  define PLUGIN_VERSION_STRING SV241PRO_STR(PLUGIN_VERSION_MAJOR)
#endif

// Numeric form returned by driverInfoVersion() (TheSkyX expects a double).
#define PLUGIN_VERSION_DOUBLE  1.0

// Git commit hash injected at build time via -DGIT_HASH=\"...\".
// Falls back to "unknown" when building outside a git tree.
#ifndef GIT_HASH
#define GIT_HASH "unknown"
#endif
