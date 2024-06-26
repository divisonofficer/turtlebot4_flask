# Parameters
# SRC_CS: The source C files to compile
# SRC_CPPS: The source CPP files to compile
# EXEC: The executable name

ifeq ($(SRC_CS) $(SRC_CPPS),)
  $(error No source files specified)
endif

ifeq ($(EXEC_SRC),)
  $(error No executable file specified)
endif

CC                  ?= gcc
CXX                 ?= g++

PUREGEV_ROOT        ?= /opt/jai/ebus_sdk/Ubuntu-22.04-x86_64
PV_LIBRARY_PATH      =$(PUREGEV_ROOT)/lib

INCLUDE_PATH        = $(PUREGEV_ROOT)/include $(SRC_HS)


CFLAGS              += -I$(INCLUDE_PATH) -I.
CPPFLAGS            += -I$(INCLUDE_PATH) -I. 

ifdef _DEBUG
    CFLAGS    += -g -D_DEBUG
    CPPFLAGS  += -g -D_DEBUG
else
    CFLAGS    += -O3
    CPPFLAGS  += -O3
endif
CFLAGS    += -D_UNIX_ -D_LINUX_ -fPIC -std=c++17
CPPFLAGS  += -D_UNIX_ -D_LINUX_ -DQT_GUI_LIB -fPIC -std=c++17

LDFLAGS             += -L$(PUREGEV_ROOT)/lib         \
                        -lPvAppUtils                 \
                        -lPtConvertersLib            \
                        -lPvBase                     \
                        -lPvBuffer                   \
                        -lPvGenICam                  \
                        -lPvStream                   \
                        -lPvDevice                   \
                        -lPvTransmitter              \
                        -lPvVirtualDevice            \
                        -lPvPersistence              \
                        -lPvSerial                   \
                        -lPvSystem                   \
                        -lPvCameraBridge



# Conditional linking and usage of the GUI on the sample only when available
ifneq ($(wildcard $(PUREGEV_ROOT)/lib/libPvGUI.so),)
    LDFLAGS   += -lPvGUI -lPvCodec
endif 

# Add simple imaging lib to the linker options only if available
ifneq ($(wildcard $(PUREGEV_ROOT)/lib/libSimpleImagingLib.so),)
    LDFLAGS   += -lSimpleImagingLib
endif

# Add CoreGEV lib to the linker options only if available
ifneq ($(wildcard $(PUREGEV_ROOT)/lib/libPvCoreGEV.so),)
    LDFLAGS   += -lPvCoreGEV
endif 

# Configure Genicam
GEN_LIB_PATH = $(PUREGEV_ROOT)/lib/genicam/bin/Linux64_x64
LDFLAGS      += -L$(GEN_LIB_PATH)


# Configure Qt compilation if any
SRC_MOC              =
MOC			         =
RCC					 =
FILES_QTGUI          = $(shell grep -l Q_OBJECT *)
ifneq ($(wildcard /etc/redhat-release),)
    QMAKE = qmake-qt5
else ifneq ($(wildcard /etc/centos-release),)
    QMAKE = qmake-qt5
else
    QMAKE = qmake
endif

ifneq ($(FILES_QTGUI),)
    # This is a sample compiling Qt code
    HAVE_QT=$(shell which $(QMAKE) &>/dev/null ; echo $?)
    ifeq ($(HAVE_QT),1)
		# We cannot compile the sample without the Qt SDK!
 		$(error The sample $(EXEC) requires the Qt SDK to be compiled. See share/samples/Readme.txt for more details)
    endif

    # Query qmake to find out the folder required to compile
    QT_SDK_BIN        = $(shell $(QMAKE) -query QT_INSTALL_BINS)
    QT_SDK_LIB        = $(shell $(QMAKE) -query QT_INSTALL_LIBS)
    QT_SDK_INC        = $(shell $(QMAKE) -query QT_INSTALL_HEADERS)

    # We have a full Qt SDK installed, so we can compile the sample
    CFLAGS 	         += -I$(QT_SDK_INC) -I$(QT_SDK_INC)/QtCore -I$(QT_SDK_INC)/QtGui -I$(QT_SDK_INC)/QtWidgets
    CPPFLAGS         += -I$(QT_SDK_INC) -I$(QT_SDK_INC)/QtCore -I$(QT_SDK_INC)/QtGui -I$(QT_SDK_INC)/QtWidgets
    LDFLAGS          += -L$(QT_SDK_LIB) -lQt5Core -lQt5Gui -lQt5Widgets

    QT_LIBRARY_PATH   = $(QT_SDK_LIB)

    FILES_MOC            = $(shell grep -l Q_OBJECT *)
    ifneq ($(FILES_MOC),)
	    SRC_MOC           = $(FILES_MOC:%h=moc_%cxx)
	    FILES_QRC         = $(shell ls *.qrc)
	    SRC_QRC           = $(FILES_QRC:%qrc=qrc_%cxx)

	    OBJS             += $(SRC_MOC:%.cxx=%.o)
	    OBJS		     += $(SRC_QRC:%.cxx=%.o)

        MOC               = $(QT_SDK_BIN)/moc
  	    RCC               = $(QT_SDK_BIN)/rcc
    endif
endif

LD_LIBRARY_PATH       = $(PV_LIBRARY_PATH):$(QT_LIBRARY_PATH):$(GEN_LIB_PATH)
export LD_LIBRARY_PATH

OBJS      += $(SRC_CPPS:%.cpp=%.o)
OBJS      += $(SRC_CS:%.c=%.o)


all: $(EXEC_SRC)

clean:
	rm -rf $(OBJS) $(SRC_MOC) $(SRC_QRC) $(EXEC_SRC:%.cpp=%.o)

moc_%.cxx: %.h
	$(MOC) $< -o $@ 

qrc_%.cxx: %.qrc
	$(RCC) $< -o $@

%.o: %.cxx
	$(CXX) -c $(CPPFLAGS) -o $@ $<

%.o: %.cpp
	$(CXX) -c $(CPPFLAGS) -o $@ $<


%.o: %.c
	$(CC) -c $(CFLAGS) -o $@ $<

$(EXEC_SRC): $(OBJS)
	echo $@ && $(CXX) -c $(CPPFLAGS) -o $(@:%.cpp=%.o) $@ && $(CXX) $(OBJS) $(@:%.cpp=%.o) -o $(@:%.cpp=%) $(LDFLAGS) 

.PHONY: all clean
