//
// Created by xuan on 10/25/22.
//

#ifndef VO_INCLUDE_ORBVOCABULARY_H_
#define VO_INCLUDE_ORBVOCABULARY_H_

#include "Thirdparty/DBoW2/DBoW2/FORB.h"
#include "Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

namespace ORB_SLAM2
{

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> ORBVocabulary;
}

#endif //VO_INCLUDE_ORBVOCABULARY_H_
