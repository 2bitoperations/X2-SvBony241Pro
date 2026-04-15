# Makefile for libx2svbony241pro

CC    = g++
RM    = rm -f
STRIP = strip

GIT_HASH := $(shell git rev-parse --short HEAD 2>/dev/null || echo unknown)

UNAME_S := $(shell uname -s)

ifneq (,$(findstring MINGW,$(UNAME_S)))
  # Windows — MSYS2 / MinGW64
  TARGET_LIB  = libx2svbony241pro.dll
  OS_FLAG     = -DSB_WIN_BUILD
  # Link runtimes statically so the DLL has no MinGW .dll dependencies.
  LDFLAGS     = -shared -static-libstdc++ -static-libgcc
  STRIP_FLAGS =
else ifeq ($(UNAME_S),Darwin)
  TARGET_LIB  = libx2svbony241pro.dylib
  OS_FLAG     = -DSB_MACOSX_BUILD
  LDFLAGS     = -dynamiclib -lstdc++
  # -x: strip local/debug symbols only; keep exported symbols so TSX can load the plugin.
  STRIP_FLAGS = -x
else
  TARGET_LIB  = libx2svbony241pro.so
  OS_FLAG     = -DSB_LINUX_BUILD
  LDFLAGS     = -shared -lstdc++ -lpthread
  STRIP_FLAGS = --strip-unneeded
endif

CPPFLAGS = -fPIC -Wall -Wextra -O2 $(OS_FLAG) -std=gnu++11 \
           -I. -Ilicensedinterfaces \
           -DX2_FLAT_INCLUDES \
           -DGIT_HASH=\"$(GIT_HASH)\"

# BUILD_NUMBER is passed from CI: make BUILD_NUMBER=42
# Locally it is omitted and version.h falls back to the bare major version.
ifdef BUILD_NUMBER
  CPPFLAGS += -DBUILD_NUMBER=$(BUILD_NUMBER)
endif

SRCS = main.cpp x2svbony241pro.cpp
OBJS = $(SRCS:.cpp=.o)

.PHONY: all clean validate_ui install udev-install

all: validate_ui ${TARGET_LIB}

validate_ui:
	@if command -v uic >/dev/null 2>&1; then \
		echo "Validating sv241pro.ui..."; \
		uic sv241pro.ui > /dev/null || (echo "UI validation failed!" && exit 1); \
		echo "UI OK."; \
	else \
		echo "uic not found, skipping UI validation."; \
	fi

${TARGET_LIB}: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^
	$(STRIP) $(STRIP_FLAGS) $@ >/dev/null 2>&1 || true

%.o: %.cpp
	$(CC) $(CPPFLAGS) -c $< -o $@

install: ${TARGET_LIB}
	./install.sh

udev-install:
	@echo "Installing udev rule for SVBony SV241 Pro..."
	sudo install -m 644 99-sv241pro.rules /etc/udev/rules.d/99-sv241pro.rules
	sudo udevadm control --reload-rules && sudo udevadm trigger
	@echo ""
	@echo "Rule installed.  Please replug the SV241 Pro USB cable."
	@echo "The device will then appear as /dev/ttyUSBSV241Pro"

clean:
	$(RM) libx2svbony241pro.so libx2svbony241pro.dylib libx2svbony241pro.dll $(OBJS)
