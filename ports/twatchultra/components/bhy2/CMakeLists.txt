# Bosch BHY2 SDK: the host-side protocol for the BHI260AP "smart sensor".
#
# TAKEN VERBATIM FROM SensorLib (github.com/lewisxhe/SensorLib, MIT for the
# wrapper) whose src/bosch/bhi260x is Bosch Sensortec's BHY2 SDK, BSD-3-Clause.
# Only the build files are ours; the sources are unmodified except for deleting
# one `#include "../../SensorBuildOpt.h"`, a SensorLib configuration header that
# has no bearing on the protocol.
#
# WHAT IT IS FOR. The BHI260AP is not an accelerometer with a register map: it
# has a Fuser core that runs algorithms (orientation, step counter, gestures,
# wake-on-motion) and reports through a FIFO. Getting a sample means speaking
# this protocol, and the protocol is what these 5,656 lines implement.
#
# IT IS NOT SMALL AND IT IS NOT FREE, but it is all flash. No RAM is allocated
# until the IMU is actually started, which is why the port uploads the firmware
# lazily.
idf_component_register(
    SRCS "bhy2.c" "bhy2_hif.c" "bhy2_parse.c" "bhy2_klio.c" "bhi3.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_common
)
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-unused-parameter -Wno-sign-compare -Wno-unused-variable
    -Wno-missing-field-initializers -Wno-unused-but-set-variable)
