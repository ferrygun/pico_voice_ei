/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * 
 * This examples captures data from an analog microphone using a sample
 * rate of 8 kHz and prints the sample values over the USB serial
 * connection.
 */
#include <stdio.h>
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "pico/stdlib.h"
#include "analog_microphone.h"
#include "analog_microphone.c"
#include "tusb.h"

static const float features_[] = {
    // copy raw features here (for example from the 'Live classification' page)
    //tr
    91, 53, 78, -50, -103, -318, 60, -27, -12, 180, -65, 75, -125, 219, 50, 155, -143, 67, -16, -63, -508, -126, 142, -125, 60, -6, 60, 71, 121, -386, 197, 47, -173, 190, 173, -7, -21, -115, 13, 88, 17, -177, 75, 81, -127, -66, 321, 223, 136, -54, 165, 246, 84, 43, -59, -175, -70, -92, -50, -223, -38, -598, 29, -42, -6, 75, 10, 167, 68, 315, 143, 4, -266, 190, -108, -95, -103, 157, -36, 150, 157, 129, 186, 130, 140, 11, 158, 26, -105, 207, -60, 39, 23, 28, 31, -107, 53, 230, 39, 15, 215, 41, 16, 36, 70, 55, 231, -16, -16, 63, 6, -138, -110, -62, -3, 96, -157, 117, 65, 281, 96, 11, 20, 92, 6, -39, -192, 41, -120, -318, 96, -30, -41, -26, -139, 121, 263, 220, 153, -678, 258, 54, 19, -16, -14, 49, 1, 148, 58, -93, 37, 177, -38, 1, 36, 186, 138, 80, -58, -798, 127, -16, 270, 123, -4, -97, -61, -148, 10, -147, -16, 269, 116, 180, 182, 186, 287, 41, -169, 92, -96, -58, -236, 102, -135, -6, 180, 134, -24, -667, 109, 147, -178, 84, 37, -679, 265, -87, -186, 17, 3, 209, 219, -16, -2, 31, 4, -303, 86, -212, 38, 19, 27, -12, -28, 86, 64, -447, 74, 55, 54, 17, 198, 37, -93, -372, 190, 76, 61, -31, 34, 116, -21, 103, 120, -71, -92, -16, -105, 47, 30, 98, -16, 122, -33, -16, 128, -45, -33, -31, -20, -5, 196, -413, 92, 365, 154, -23, -28, 45, 22, 49, -16, -173, 85, -92, 156, 246, 70, -211, 15, 121, 23, -161, 97, 146, -15, -454, 127, 23, -7, 3, -82, -7, 247, 159, -77, 86, -125, -57, -126, -134, 58, 4, -285, 58, -160, 211, -114, 95, -48, 99, 98, 101, -98, 76, 74, 143, -20, -93, -23, 303, 200, 109, 236, 128, 19, -110, -186, -113, -47, 71, 36, 144, 228, -269, -35, -23, 97, 79, 113, 118, 252, 71, 49, -63, -76, -111, -812, -16, -71, -4, 159, -25, 150, -245, 67, -16, -509, 53, 129, 87, 10, 44, 218, -63, -307, 307, 43, -16, -590, 104, 67, -261, -34, 107, -86, 70, 74, 35, 34, 7, -693, -44, -117, 42, 3, 49, 61, -4, -211, -416, -619, 155, 121, -5, 207, 53, 274, 247, 53, 123, 82, -92, -52, 38, 142, 151, 103, 169, 66, 189, 165, 89, 119, 63, 33, 230, 53, 55, -721, 45, -126, 77, 86, 46, 115, 298, 45, 13, 42, 114, 62, 25, 13, 148, -17, 135, 67, -23, -63, -124, 34, -28, -63, 84, -108, 185, 56, -35, 180, 100, -609, -27, -39, 61, 46, -39, -95, -50, -379, -25, -79, 155, 94, -216, -524, 154, 39, -99, 157, 8, 68, -13, 82, 5, -743, -84, -16, 157, 9, 48, 167, 111, 107, -44, 71, 126, -30, -433, 135, -26, 174, -21, -54, -3, 33, 0, 165, 27, -30, 314, 102, 58, 257, 198, 77, -16, 166, 20, 151, -46, -93, 6, 38, 33, 90, 44, 8, 87, -7, 415, 115, 35, 118, 34, 188, 81, 44, -717, 185, 38, -63, -648, 156, 22, 38, 38, 37, 189, -16, -494, 278, 93, -98, -57, 4, 15, -29, 5, -87, 31, -99, -87, -16, -195, 255, 103, 100, -13, 160, 212, 125, 115, -1, 125, 29, -9, 53, -114, 27, 83, 165, 247, 62, 43, -73, -67, 242, 147, 140, -155, 86, -139, -82, -131, -42, -204, 54, 193, 73, 7, 272, 71, 39, 14, -3, 171, 121, -11, 132, 111, 212, -54, -33, 22, -163, -589, 25, -641, 350, 86, 133, -16, 125, 22, 57, -16, 223, -9, 53, 79, 180, -43, -348, -437, -157, 253, 289, -16, 133, -249, 498, 250, -309, -304, 410, -87, 168, 835, -516, 522, -158, 101, 12, -20, 949, -370, 108, 202, -279, -15, 33, -302, 871, -263, 85, 329, -161, -147, -226, 832, -415, 404, 33, 65, -132, 26, -538, 605, -258, 365, -97, -287, 35, -98, 102, -32, 22, -47, -22, 19, 34, 62, -300, 313, -174, -21, -217, 153, 38, -179, 385, 27, 43, -158, 317, -16, -228, -237, 273, -82, -25, -177, -111, 6, -239, 237, -315, 271, -340, -69, 45, -249, 300, 564, -521, 505, 164, 50, 62, -525, 510, -423, 273, -167, 213, 202, -204, 377, -462, 177, 27, -100, -16, 67, -116, 437, -316, 123, -182, 77, 110, -64, 428, -478, 214, 311, -413, -121, -174, 161, -374, -63, 435, -213, -212, 66, -25, -450, -81, 426, -183, -65, -131, 5, -101, -484, -165, 347, -282, 65, 75, -348, -602, 158, 350, -169, 324, 218, 7, -334, -527, 549, 357, -31, 401, -13, -428, -217, 935, -572, 113, 353, 115, 162, -452, 869, 385, -656, 113, 297, -1, -600, 1019, -130, -626, 293, 263, -75, 841, 885, 392, -253, -506, 273, 240, -582, 640, 10, -589, -635, 197, -259, -377, 349, -73, -121, -346, 257, 139, -75, -220, 43, 62, -466, 140, 87, -245, -223, -124, -122, 114, -87, 153, -63, -75, 115, 28, 71, -30, 65, -238, -127, 58, -135, -131, 159, 76, 127, 43, -107, 28, -48, 41, 266, 151, -16, -20, 5, 144, -156, 127, 98, 145, 139, -70, -4, 141, -54, -123, 193, 70, 19, -43, 55, 110, 45, 58, -254, -70, -25, -114, -169, 94, 45, 99, -50, 78, 1, -11, 292, 258, -142, -158, -229, -79, -89, -166, -51, -50, 74, -198, -61, -245, 132, 13, 181, 178, -328, -59, 191, 127, 228, -165, 54, 31, -468, -140, -107, 199, -369, -51, 44, -83, 11, 90, 23, 58, -3, 117, -85, 90, -133, -89, -70, 132, -184, 83, -16, 142, 151, 126, -86, 155, 70, -15, 78, 102, -3, -23, -109, 201, -447, -13, 65, 1, -160, -102, 429, 15, -14, 66, -7, 179, -54, 107, 514, -42, 130, 10, -76, -137, -94, -197, -73, -91, -4, -16, 412, -67, 195, 92, -110, -234, -8, -58, -169, -221, -548, 649, -279, 285, 351, -76, 522, 337, -115, 191, -16, -110, -59, -166, -368, 80, -405, 733, -474, -22, 63, 459, -148, 220, -66, -162, -171, 822, -131, 338, -495, 329, 189, -374, 31, 166, 137, 42, -210, 49, -285, 401, -25, 93, -283, 177, 33, 139, -421, 59, -20, -317, -16, -117, 12, -2, -146, 149, -16, 226, 210, 86, 28, -25, -178, -28, -304, 317, 302, -47, 177, 53, -135, 145, -124, -335, 149, -17, 64, -16, 33, 34, -109, 79, 77, 164, 155, -76, -342, 58, -64, 35, 81, 96, 22, -196, 50, 145, -10, 54, 69, -161, 53, 11, 33, -16, -52, -159, 343, -53, -38, 182, -39, -317, 53, -27, 261, 41, 27, 54, -233, -290, -129, -281, 343, -247, 61, -5, -7, -36, -109, 447, 483, 48, 130, 404, -9, -167, -270, -69, -88, 196, 405, -172, -254, -163, -649, 149, 122, 223, 63, -81, 179, -331, -239, 362, 100, -315, 21, 150, -443, 841, 313, -25, 73, 43, 350, 55, -82, 647, 156, -266, 61, 564, -83, -393, -255, -496, -121, -177, 327, -13, -75, -257, -246, -352, 332, -142, 214, 256, -16, -452, 524, 12, 12, 227, 165, 51, 25, -183, 681, 268, 212, 403, -33, -84, -178, -102, -142, -235, 321, -156, -185, -282, -273, -374, 207, -10, 98, 39, -92, -283, 405, 337, 379, -65, 99, 73, 143, -86, 644, 469, 42, 171, 186, 193, 12, -293, 83, -142, -13, 71, 96, -60, -86, -554, 269, -238, -417, 360, -79, -280, 677, 29, -209, -279, 270, -37, -20, 251, 558, -169, -647, -269, 108, -301, -379, 306, 18, -190, -121, 50, -55, -25, 333, 162, 195, -19, 31, 159, -231, -114, 286, 373, 1, 45, -9, 49, -233, 14, 319, 283, -123, 84, -16, 161, -107, -63, 353, 155, -103, -243, 37, -108, -93, 398, 217, -388, -59, -110, 219, 238, -6, -27, 181, -174, -515, 111, -38, -54, -14, -127, 25, -117, 34, 115, 152, 73, 12, 93, 43, -18, -90, 182, -14, -206, 17, -16, -45, 38, 4, 191, 250, 147, 0, 257, 147, 67, 202, 110, -279, -83, -23, 78, -258, -241, 51, 191, -117, -6, 146, 231, 90, 139, 185, -96, -33, 76, -6, -165, -222, -116, -12, -117, -118, 109, 118, -3, 18, 159, 247, 9, 96, 108, -42, -191, -6, -195, 26, -176, -25, 18, -465, -59, 75, 175, 95, 93, 110, 109, -189, 89, 154, -13, -170, -123, -107, -1, -94, 19, 174, -379, 100, 108, 102, 287, 102, 148, -57, -69, -30, -41, -106, -113, -67, 13, -109, 22, 35, 56, -11, 106, 162, -343, -243, 63, 37, 163, 64, 13, -429, 99, -85, -112, -21, 23, -1, 6, 41, 105, -3, 164, -89, 109, 149, 26, -22, 125, 20, 66, -549, -38, -66, -139, 33, 41, -87, -39, 22, -190, 163, 135, 85, -331, -239, -282, -252, 310, 201, -108, 236, -132, 83, -436, -1073, 213, 114, 532, -44, 507, -68, -708, 716, -483, 347, -405, 73, -138, -267, -358, -180, 105, -581, -578, 307, -91, 298, -554, 502, -211, 426, -886, 363, -197, -659, 699, 762, -315, 427, 90, -188, 21, 183, -391, 790, -190, 667, -150, 45, 29, -165, -446, -292, 306, 477, -373, 99, -51, 94, -358, 110, -66, 462, -350, 321, -185, 125, -379, 398, -279, 44, 183, -334, 255, -31, -362, 621, -218, 401, 143, -174, -334, 54, -365, -703, 569, -503, -417, -214, -153, -59, -210, 269, -434, 484, -534, 459, -5, 28, 333, 390, -177, -89, 199, -540, 220, 125, -428, 691, -290, 532, 37, 2, 124, 33, -385, -659, 537, -513, 583, -404, -14, -31, -441, 26, 5, -484, 499, 347, -67, 292, -299, -244, 778, -609, 281, -432, 129, -229, -55, -373, -292, 635, 33, -16, 28, -15, -153, -113, 205, -503, 753, -325, 150, 97, -461, 348, -227, -286, 484, 300, 41, -16, -455, -593, 874, -564, 11, -183, 150, -27, -213, -59, 444, 605, -290, -16, 78, 92, -724, -455, -182, -555, 463, -258, 47, -66, -484, 846, -537, 377, 159, 63, 62, 139, 76, 257, 275, -521, 594, -130, 60, -22, -531, 735, -486, -16, 229, -150, 68, -118, -218, -381, 608, -410, 174, -209, -46, -4, -577, 372, -330, 406, -211, -9, -124, 12, -277, 873, -374, 387, 13, -17, -79, 147, -367, 383, -413, 396, -148, 183, 21, -347, 497, 565, -201, 125, 145, -7, 73, -122, -96, 277, -260, 370, -142, -67, 175, -217, 336, 289, 120, 51, -60, -57, -35, 119, 40, 309, -235, 131, -299, -81, -16, -75, 119, 138, 91, -300, -245, 58, -13, 109, -7, 316, -16, 67, 83, -139, 89, -33, -47, -59, 109, -543, -73, -6, 65, 84, 7, 193, 197, -1, -16, -190, -26, 46, -504, -58, 117, -41, -67, -234, 27, 129, -18, 256, 126, -56, 46, 23, -107, -285, -114, -180, -271, -6, 187, -6, -21, 124, 74, -1, 99, -221, -16, -39, -73, 170, -70, 41, 49, -50, -86, -79, -16, -29, -331, 29, 118, -241, 21, 77, 152, 6, 58, 91, -78, 109, -17, -198, 44, -45, 11, -38, -76, 29, -23, 95, 0, -83, -180, 109, -19, 125, 113, -29, -23, -107, -133, 103, 25, -23, 95, -303, 252, 62, 29, 154, 51, 67, -38, -61, -25, 145, 1, -139, 128, 100, -62, -89, 22, 22, -14, -68, 61, -142, -251, 34, 95, -327, 231, -33, 61, 4, 79, 73, -172, 107, 70, -507, 71, 35, 109, -37, 108, -81, 54, 142, 65, -16, -507, 44, -161, -25, 99, 173, 90, 26, -17, -306, -11, -111, 61, 121, 10, -29, -157, 105, 167, -25, -442, 183, -116, -39, 94, -10, 30, -33, -28, -7, -415, 117, 62, -308, 252, -39, 63, -318, 125, -16, 283, -328, 243, -9, -356, 81, 42, -507, -11, 58, -23, 162, 42, -212, 77, 33, -44, 38, -852, 78, 313, 46, -65, -252, -55, -124, 50, 77, 63, -69, 234, 95, 66, 74, 74, 102, 134, 101, -124, -16, -16, -447, 139, -252, 294, 149, 54, 64, -116, 126, 85, -639, 198, -38, -31, 52, 111, -78, 53, 110, 164, -16, -105, 151, -266, 251, -103, 60, 62, 12, -449, 231, 155, 71, 312, 172, 17, 226, -65, -22, 116, -25, -15, -399, 57, -11, 123, 81, 1, 68, 130, 21, 270, 143, 71, -38, -88, -141, -25, -547, -39, -23, 82, -100, 33, -39, 42, -625, 126, 190, 168, 274, 21, 181, -84, -455, -21, 130, 46, -70, -629, -22, 151, 201, 173, 365, 81, 26, -46, 144, 18, -6, 78, -20, -13, 27, 270, 119, 241, -319, -16, 58, -15, 34, -43, -244, -50, 89, -133, -27, -39, -271, -254, 96, -183, -190, 143, -70, 82, 53, 18, 10, 242, 54, -428, 159, 152, 17, -163, -267, 52, 195, -33, -666, 257, 97, 58, -26, -102, 117, 29, 101, -78, -431, -134, -22, 60, 5, -244, -143, 23, -12, -59, 142, 125, -270, -107, 130, 110, -47, -30, 141, 230, 26, 145, -18, -8, 99, -95, -603, -92, -78, 69, 197, 42, 111, -77, -210, 40, -333, 51, -14, 265, -35, 22, 170, 42, -27, -572, 106, 129, 52, 210, 58, -39, -16, -182, -134, 317, -710, 234, 268, 161, 188, 214, -174, 211, -71, -170, 174, 36, 27, 59, 74, -52, -639, 145, 53, 87, -169, -18, 85, 163, 188, 24, -2, -91, -80, -147, -159, -87, -353, -205, -148, 12, -16, -1, 107, 169, -315, 47, -87, -2, 73, 119, 2, -726, -97, 6, 46, 34, -132, 67, 85, 332, 395, 285, 226, 115, 167, 55, 171, 93, -116, -187, -122, -86, 86, 55, 76, 208, -1, -667, 359, 142, 233, 154, -30, -92, -81, -202, 8, 12, -147, -159, 63, -22, 46, 193, -468, -5, 155, 45, 169, 105, 136, -90, -66, -147, -74, 62, -122, -295, 12, 73, 269, 195, 224, 180, 193, 269, 118, 157, 25, -30, -102, -158, 43, -34, 17, 9, 106, 358, 18, 38, 233, 227, -16, -12, 17, 59, 330, 176, 50, 135, -47, 43, -16, 20, -43, -3, 14, -3, 86, 114, 257, -466, 100, 107, 123, -16, 35, 95, 7, -109, 30, -115, -342, -50, 65, 147, -16, -387, 263, 82, 14, 113, -182, 42, 93, -53, 1, -23, 140, 86, -34, 172, 206, -37, -125, 34, 38, 76, 11, 21, 124, 110, 109, 146, 58, 121, 9, -91, -93, 13, -3, -801, -25, -363, 347, 119, 80, 158, 68, -459, -32, 49, -183, 10, -130, -16, 164, -93, 39, 202, -13, -213, 177, 97, 65, -195, 31, 146, -39, 23, -51, -655, 262, 63, 175, -2, -3, 74, 51, 153, 132, 31, -413, 67, -639, -6, -54, -202, 131, -4, 74, 98, -365, 126, -83, 51, -102, 42, -49, 122, 179, 139, 203, 131, 223, 2, -24, -59, -109, -438, 9, 54, 78, -351, -37, 67, 297, 82, 122, 114, 135, -17, 306, 17, -7, -65, -16, 32, 44, 93, -181, 79, -19, -13, 22, 23, 36, 90, 139, 74, 60, 177, -15, -28, 95, -16, 13, 51, 5, 43, 108, -633, 225, 83, 25, 29, 50, -21, 278, 37, 75, 11, -332, -35, 82, -71, -173, 134, 131, 74, -143, 13, 75, 77, 27, 189, -16, -40, -16, -734, 58, 108, 18, -475, -106, 171, 153, 126, 138, -60, 158, -47, -83, -699, -53, -20, 55, 135, -74, -54, 34, 178, -1, 143, 155, 274, 126, -39, 135, -245, -775, -84, -31, 84, -604, 239, -26, 43, -164, 82, 123, 207, 267, 71, 165, -267, 20, -134, 38, -115, 19, -40, -7, 42, 238, 85, 122, 160, -193, -19, 57, 110, 155, 82, 65, 257, -111, -571, -266, -460, -30, 206, 138, 156, 186, 101, 10, -345, 82, 113, 120, 44, 154, -219, 77, -92, 19, -198, 46, 170, 95, 111, 84, -202, 65, 10, 29, 109, 25, 244, -741, -16, -18, 38, -166, -393, -122, 123, 154, 79, 156, 285, 41, 108, -529, -35, 365, 239, 96, 143, 21, 2, -155, 75, -47, -127, -15, -16, -326, -45, 44, 79, 108, -58, -178, 263, 137, 52, -611, -181, -814, -77, -176, -165, -194, 70, 195, 207, 163, 225, -260, -39, 35, -177, -632, 195, 148, 81, 148, 41, -15, 122, 79, -633, 172, 52, -31, 220, 267, -6, 9, -545, 132, 153, 356, -55, 23, -16, 123, 8, -21, -76, -285, 116, 239, 154, 134, 42, -13, -91, 59, -156, 37, 3
};

// configuration
#define INSIZE 16

const struct analog_microphone_config config = {
    // GPIO to use for input, must be ADC compatible (GPIO 26 - 28)
    .gpio = 26,

    // bias voltage of microphone in volts
    .bias_voltage = 1.25,

    // sample rate in Hz
    .sample_rate = 16000,

    // number of samples to buffer
    .sample_buffer_size = INSIZE,
};

// variables
int16_t sample_buffer[INSIZE];
volatile int samples_read = 0;

float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

const int minutes = .3;
const int threshold = 10;
const float thresh = 0.9;

int count_label_on = 0;


void on_analog_samples_ready() {
    // callback from library when all the samples in the library
    // internal sample buffer are ready for reading 
    samples_read = analog_microphone_read(sample_buffer, INSIZE);
}


int raw_feature_get_data(size_t offset, size_t length, float * out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
}

void wait_for_usb() {
    while (!tud_cdc_connected()) {
        printf(".");
        sleep_ms(500);
    }
    printf("usb host detected\n");
}   

int main(void) {
    // initialize stdio and wait for USB CDC connect
    stdio_init_all();

    wait_for_usb();

    printf("connected\n");

        printf("size of features/floats: %d\n", sizeof(features) / sizeof(float));
        printf("EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE:%d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
        printf("EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME:%d\n", EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME);
        printf("size of features returned: %d\n", sizeof(features));
        printf("total_length: %d\n", sizeof(features) / sizeof(features[0]));
        printf("\n");



    ei_impulse_result_t result = {
        nullptr
    };

    ei_printf("Edge Impulse standalone inferencing (Raspberry Pi Pico)\n");

    // initialize the analog microphone
    if (analog_microphone_init( & config) < 0) {
        ei_printf("analog microphone initialization failed!\n");
        while (1) {
            tight_loop_contents();
        }
    }

    // set callback that is called when all the samples in the library
    // internal sample buffer are ready for reading
    analog_microphone_set_samples_ready_handler(on_analog_samples_ready);

    // start capturing data from the analog microphone
    if (analog_microphone_start() < 0) {
        ei_printf("Analog microphone start failed!\n");
        while (1) {
            tight_loop_contents();
        }
    }

    /*
    if (sizeof(features) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE)
    {
      ei_printf("The size of your 'features' array is not correct. Expected %d items, but had %u\n",
                EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(features) / sizeof(float));
      return 1;
    }
    */
    
    while (1) {
        // wait for new samples
        while (samples_read == 0) {
            tight_loop_contents();
        }

        // store and clear the samples read from the callback
        int sample_count = samples_read;
        samples_read = 0;
        

        // loop through any new collected samples
        
        for (int i = 0; i < sample_count; i++) { 
            features[i] = (float)sample_buffer[i];
            
        }
        

        printf("sample_count returned: %d\n", sample_count);
        printf("size of features/floats: %d\n", sizeof(features) / sizeof(float));
        printf("EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE:%d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
        printf("EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME:%d\n", EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME);
        printf("size of features returned: %d\n", sizeof(features));
        printf("total_length: %d\n", sizeof(features) / sizeof(features[0]));
        printf("\n");

        // the features are stored into flash, and we don't want to load everything into RAM
        signal_t features_signal;
        features_signal.total_length = sizeof(features) / sizeof(features[0]);
        features_signal.get_data = &raw_feature_get_data;


        // invoke the impulse
        EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);

        //ei_printf("run_classifier returned: %d\n", res);

        if (res != 0)
            return 1;

        ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);

        // print the predictions
        ei_printf("[");
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            ei_printf("%s: %.5f", result.classification[ix].label, result.classification[ix].value);

           
            #if EI_CLASSIFIER_HAS_ANOMALY == 1
            ei_printf(", ");
            #else
            if (ix != EI_CLASSIFIER_LABEL_COUNT - 1) {
                ei_printf(", ");
            }
            #endif
        }
        #if EI_CLASSIFIER_HAS_ANOMALY == 1
        printf("%.3f", result.anomaly);
        #endif
        printf("]\n");
        
        

        ei_sleep(2000);

    }

    return 0;
}
