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
响应外部sdk接口规划
=====================================================*/
#pragma once
#include"plan.h"
#include<array>

namespace Plan{

class maniPlanClass final:public basePlanClass{
public:
	maniPlanClass();
	void init(float dt)override final;
	void hey(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef)override final;
	bool run(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef)override final;
	void bye(const Nabo::inputStruct &inRef)override final;
	void log() override final;
private:
	void update();
	void planArm();
	void planNeck();
	void planLumbar();
	void planFinger();
	void dwdate(Nabo::outputStruct &outRef);
	const vec3f eyePos{0.08,0,0.6};//大概值
	int filtLevel;
	
	int startCnt;
	enum ArmMode{
		ArmNone=0,
		ArmRc=1,
		ArmRoute=2,//透传下肢关节命令
		ArmJoint=3,
		ArmCart2B=4,
		ArmCart2F=5,
	};
	enum NeckMode{
		NeckNone=0,
		NeckRc=1,
		NeckRoute=2,
		NeckJoint=3,
		NeckNavi=4,
		NeckLeftHand=5,
		NeckRightHand=6,
		NeckMidHand=7,
	};
	enum LumbarMode{
		LumbarNone=0,
		LumbarRc=1,
		LumbarRoute=2,
		LumbarJoint=3,
		LumbarRot=4,
	};
	enum FingerMode{
		FingerNone=0,
		FingerRc=1,
		FingerRoute=2,
		FingerJoint=3,
		FingerStraight=4,
	};
	struct{
		ArmMode arm;
		NeckMode neck;
		LumbarMode lumbar;
		FingerMode finger;
		float armFilRate, neckFilRate, lumbarFilRate, fingerFilRate;//用于模式改变安全滤波
		void reset(){
			arm   =ArmNone;
			neck  =NeckNone;
			lumbar=LumbarNone;
			finger=FingerNone;
			armFilRate=0;
			neckFilRate=0;
			lumbarFilRate=0;
			fingerFilRate=0;
		}
	}ctrlMode;

	enum{
		Stop,Start
	}actn;
};
}//namespace
