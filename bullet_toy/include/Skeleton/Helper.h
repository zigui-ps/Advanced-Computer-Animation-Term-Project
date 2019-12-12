#ifndef HELPER
#define HELPER

#include "Skeleton/Skeleton.h"

void rotateY(SkeletonPosePtr pose, double dY);
double getYrot(SkeletonPosePtr from, SkeletonPosePtr to);
double getYrot(Eigen::Quaterniond from, Eigen::Quaterniond to);
SkeletonPosePtr getPoseDifference(SkeletonPosePtr to, SkeletonPosePtr from);

#endif
