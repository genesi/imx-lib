ifeq ($(BOARD_HAVE_VPU),true)

LOCAL_PATH := $(call my-dir)

# Share library
include $(CLEAR_VARS)
LOCAL_SRC_FILES := \
	vpu_io.c \
	vpu_util.c \
	vpu_lib.c
LOCAL_CFLAGS += -DBUILD_FOR_ANDROID -D$(BOARD_SOC_TYPE)
LOCAL_C_INCLUDES += $(LOCAL_PATH)
LOCAL_MODULE := libvpu
LOCAL_LD_FLAGS += -nostartfiles
LOCAL_PRELINK_MODULE := false
include $(BUILD_SHARED_LIBRARY)

endif
