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
运动规划基类
继承提供统一接口，可方便做管理调度（todo：manager.cpp内实现系统调度）
继承会导致阅读不便，基类尽量不添加复杂逻辑
=====================================================*/
#pragma once
#include"iopack.h"
#include"robot_arm.h"
#include"log.h"
#include"fsm.h"

namespace Plan{
class basePlanClass{
public:
	basePlanClass();
	virtual void init(float dt);
	virtual void hey(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef)=0;
	virtual bool run(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef)=0;
	virtual void bye(const Nabo::inputStruct &inRef)=0;
	virtual void log()=0;
protected:
	//都会用到的变量，在基类定义。个体plan用到的变量在个体plan中定义即可
	string planName;//manager内添加到fsm时给定
	Rbt::armClass &arm=Rbt::armClass::instance();
	float dt;
	float tim;
	bool quitFlag;
	bool logFlag,consoleFlag;
	int logCnt,consoleCnt;

	Nabo::cmdStruct inCmd;
	Nabo::maniSdkStruct inManiSdk;
	Nabo::loco2maniStruct inLocoCmd;
	Nabo::sensorStruct inSens;
	Nabo::ctrlStruct outCtrl;

	vecNf(NMotMain) j0;
	vecNf(NMotFingerBoth) finger0;
	Alg::filterOneClass jFil[NMotMain];
	// Alg::filterOneClass<float> jFil[NMotAll];
	// Alg::filterTwoClass jFil[NMotAll];
	
	Log::logClass logger;
	stringstream logSS;
	// Alg::filterOneClass<float> wFil[NMotAll],tInFil[NMotAll],tOutFil[NMotAll];
	Alg::filterOneClass wFil[NMotMain],tInFil[NMotMain],tOutFil[NMotMain];
	//base开头的函数所有plan都会调用，标记为fianl
	virtual void baseReset(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef)final;
	virtual void baseRun(const Nabo::inputStruct &inRef,Nabo::outputStruct &outRef)final;
	virtual const vecNf(NMotMain) &getJntStd()final{return jntStd;}
	virtual const vecNf(NMotMain) &getJntKpInit()final{return jntKpInit;}
	virtual const vecNf(NMotMain) &getJntKdInit()final{return jntKdInit;}
	virtual const vecNf(NMotMain) &getJntMaxTorInit()final{return jntMaxTorInit;}
	virtual const vecNf(NMotFingerBoth) &getFingerStd()final{return fingerStd;}
	virtual const float &getFingerMaxTorInit()final{return fingerMaxTorInit;}
	friend class Mng::fsmClass;
private:
	//防止被意外更改，使用时通过上面的get获取引用
	vecNf(NMotMain) jntStd,jntKpInit,jntKdInit,jntMaxTorInit;
	vecNf(NMotFingerBoth) fingerStd;
	float fingerMaxTorInit;
};
}//namespace
