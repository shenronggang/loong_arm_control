/*
Copyright 2025 国家地方共建人形机器人创新中心/人形机器人（上海）有限公司, https://www.openloong.net
Thanks for the open biped control project Nabo: https://github.com/tryingfly/nabo

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

============ ***doc description @ yyp*** ============
常量定义，首字母大写
#include"nabo_nabo_config.h"
关节顺序：左臂7*2 头2 腰3 左右指6*2
=====================================================*/
#pragma once

//涉及数组大小，必须编译前给定
static const int NArmDof       =7;
static const int NNeckDof      =2;
static const int NLumbarDof    =3;

static const int NMotArm       =NArmDof*2;
static const int NMotNeck      =NNeckDof;
static const int NMotLumbar    =NLumbarDof;
static const int NMotMain      =NMotArm +NNeckDof +NLumbarDof;
static const int NMotFingerLeft=6;
static const int NMotFingerRight=6;
static const int NMotFingerBoth=NMotFingerLeft+NMotFingerRight;


// 左臂7 右臂7 头2 腰3
static const int IdArm[2]   {0, 7};
static const int IdNeck     {14};
static const int IdLumbar   {16};
