#include <jni.h>
#include <vector>
#include <cmath>
#include "apriltag.h"
#include "tag36h11.h"
#include "apriltag_pose.h"

extern "C" {

struct NativeDetection {
    int id;
    double tx, ty, tz;
    double r00, r01, r02;
    double r10, r11, r12;
    double r20, r21, r22;
};

static apriltag_detector_t* td = nullptr;
static apriltag_family_t* tf = nullptr;

void init_detector() {
    if (!td) {
        tf = tag36h11_create();
        td = apriltag_detector_create();
        apriltag_detector_add_family_bits(td, tf, 1);
        td->quad_decimate = 1.0;
        td->nthreads = 4;
    }
}

__attribute__((visibility("default")))
int detect_tags(uint8_t* gray_bytes, int width, int height, double fx, double fy, double cx, double cy, double tag_size, NativeDetection* out_detections, int max_detections) {
    init_detector();

    image_u8_t img = {
            .width = width,
            .height = height,
            .stride = width,
            .buf = gray_bytes
    };

    zarray_t* detections = apriltag_detector_detect(td, &img);
    int num_detected = zarray_size(detections);
    int count = 0;

    for (int i = 0; i < num_detected && count < max_detections; i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        apriltag_detection_info_t info = {
                .det = det,
                .tagsize = tag_size,
                .fx = fx,
                .fy = fy,
                .cx = cx,
                .cy = cy
        };

        apriltag_pose_t pose;
        estimate_tag_pose(&info, &pose);

        out_detections[count] = NativeDetection{
                .id = det->id,
                .tx = pose.t->data[0],
                .ty = pose.t->data[1],
                .tz = pose.t->data[2],
                .r00 = pose.R->data[0], .r01 = pose.R->data[1], .r02 = pose.R->data[2],
                .r10 = pose.R->data[3], .r11 = pose.R->data[4], .r12 = pose.R->data[5],
                .r20 = pose.R->data[6], .r21 = pose.R->data[7], .r22 = pose.R->data[8]
        };

        matd_destroy(pose.t);
        matd_destroy(pose.R);

        count++;
    }

    apriltag_detections_destroy(detections);
    return count;
}

}