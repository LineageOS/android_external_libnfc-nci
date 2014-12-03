# Copyright (C) 2011 The Android Open Source Project
# Copyright (C) 2014 Cyanogen Inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# function to find all *.cpp files under a directory
define all-cpp-files-under
$(patsubst ./%,%, \
  $(shell cd $(LOCAL_PATH) ; \
          find $(1) -name "*.cpp" -and -not -name ".*") \
 )
endef

ifneq ($(BOARD_NFC_HAL_SUFFIX),)
    HAL_SUFFIX := pn54x.$(BOARD_NFC_HAL_SUFFIX)
else
    HAL_SUFFIX := pn54x.default
endif

LOCAL_PRELINK_MODULE := false
LOCAL_ARM_MODE := arm
LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE := nfc_nci.$(HAL_SUFFIX)
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_SRC_FILES := $(call all-c-files-under, .)  $(call all-cpp-files-under, .)
LOCAL_SHARED_LIBRARIES := liblog libcutils libhardware_legacy libdl libstlport
LOCAL_MODULE_TAGS := optional

LOCAL_C_INCLUDES += external/stlport/stlport  bionic/  bionic/libstdc++/include \
	$(LOCAL_PATH)/utils \
	$(LOCAL_PATH)/inc \
	$(LOCAL_PATH)/common \
	$(LOCAL_PATH)/dnld \
	$(LOCAL_PATH)/hal \
	$(LOCAL_PATH)/log \
	$(LOCAL_PATH)/tml \
	$(LOCAL_PATH)/self-test

LOCAL_CFLAGS += -DANDROID \
        -DNXP_UICC_ENABLE -DNXP_HW_SELF_TEST
#LOCAL_CFLAGS += -DFELICA_CLT_ENABLE
#-DNXP_PN547C1_DOWNLOAD

LOCAL_ADDITIONAL_DEPENDENCIES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr
LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
include $(BUILD_SHARED_LIBRARY)
