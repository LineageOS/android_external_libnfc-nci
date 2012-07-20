LOCAL_PATH:= $(call my-dir)
ASTL:=../astl
NFA:=src/nfa
NFC:=src/nfc
UDRV:=src/udrv

#gki
GKI_FILES:=$(call all-c-files-under, src/gki)

#
# libnfc-ntal
#
NTAL_FILES:= src/ntal/userial_linux.c

#
# app_logging
#
LOG_FILES:= \
	    src/adaptation/bte_logmsg.c \
	    src/adaptation/android_logmsg.cpp

#
# libnfc-nci
#
#nfa
NFA_ADAP_FILES:= \
    src/adaptation/config.cpp \
    src/adaptation/defconfig.c \
    src/adaptation/NfcAdaptation.cpp \
    src/adaptation/patchram.cpp \
    	src/adaptation/libmain.c

#nfa
NFA_FILES:= $(call all-c-files-under, $(NFA))
NFC_FILES:= $(call all-c-files-under, $(NFC))

COMMON_CFLAGS += -DANDROID -DANDROID_USE_LOGCAT=TRUE

NFA_CFLAGS += -DNFC_CONTROLLER_ID=1
D_CFLAGS += -DDEBUG -D_DEBUG -O0 -g

ifeq ($(findstring maguro,$(TARGET_PRODUCT)),)
ifeq ($(findstring crespo,$(TARGET_PRODUCT)),)
D_CFLAGS += -DGENERIC_TARGET=1
endif
endif

D_CFLAGS += -DBUILDCFG=1
D_CFLAGS += -DNFA_APP_DOWNLOAD_NFC_PATCHRAM=TRUE

#Adaptation include
NFA_CFLAGS += -I$(LOCAL_PATH)/$(ASTL)/include

#NTAL includes
NTAL_CFLAGS += -I$(LOCAL_PATH)/src/include
NTAL_CFLAGS += -I$(LOCAL_PATH)/src/gki/ulinux
NTAL_CFLAGS += -I$(LOCAL_PATH)/src/gki/common

#NFA NFC includes
NFA_CFLAGS += -I$(LOCAL_PATH)/src/include
NFA_CFLAGS += -I$(LOCAL_PATH)/src/gki/ulinux
NFA_CFLAGS += -I$(LOCAL_PATH)/src/gki/common
NFA_CFLAGS += -I$(LOCAL_PATH)/$(NFA)/brcm
NFA_CFLAGS += -I$(LOCAL_PATH)/$(NFA)/include
NFA_CFLAGS += -I$(LOCAL_PATH)/$(NFA)/int
NFA_CFLAGS += -I$(LOCAL_PATH)/$(NFC)/brcm
NFA_CFLAGS += -I$(LOCAL_PATH)/$(NFC)/include
NFA_CFLAGS += -I$(LOCAL_PATH)/$(NFC)/int
NFA_CFLAGS += -I$(LOCAL_PATH)/$(UDRV)/include

ifneq ($(NCI_VERSION),)
NFA_CFLAGS += -DNCI_VERSION=$(NCI_VERSION)
endif

include $(CLEAR_VARS)
LOCAL_PRELINK_MODULE := false
LOCAL_ARM_MODE := arm
LOCAL_MODULE:= libnfc-nci
LOCAL_MODULE_TAGS := optional
LOCAL_SHARED_LIBRARIES := libhardware_legacy libcutils libdl
LOCAL_STATIC_LIBRARIES := libastl
LOCAL_CFLAGS := $(COMMON_CFLAGS) $(D_CFLAGS) $(NFA_CFLAGS)
LOCAL_SRC_FILES := $(NFA_ADAP_FILES) $(GKI_FILES) $(NFA_FILES) $(NFC_FILES) $(NTAL_FILES) $(LOG_FILES)
include $(BUILD_SHARED_LIBRARY)

