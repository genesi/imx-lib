LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
       mxc_ipu_hl_lib.c \
       mxc_ipu_lib.c

LOCAL_CFLAGS += -DBUILD_FOR_ANDROID

LOCAL_C_INCLUDES += $(LOCAL_PATH)

LOCAL_SHARED_LIBRARIES := libutils libc

LOCAL_MODULE := libipu
LOCAL_LD_FLAGS += -nostartfiles
LOCAL_PRELINK_MODULE := false
include $(BUILD_SHARED_LIBRARY)

