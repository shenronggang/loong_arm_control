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
先左后右
=====================================================*/
#pragma once
#include"robot_config.h"
#include"eigen.h"


namespace Rbt{
class armClass{
public:
	static armClass& instance();
	void setDt(float dt);
	// void update(vecNf(NArmMot)&armQ, vecNf(NArmMot)&armQd, vecNf(NArmMot)&armTau);
	void update(float *armQ, float *armQd, float* armTau);
	const vec3f &getTipP(const int &armId);
	const mat3f &getTipR(const int &armId);
	const vec3f &getTipRpy(const int &armId);
	const vec6f &getTipVW(const int &armId);
	const vec6f &getTipFM(const int &armId);

	const vec3f &getTgtTipP(const int &armId);
	// const mat3f &getTgtTipR(const int &armId);//因存在cmd末端与urdf末端定义的转换，该结果为urdf坐标系的表达
	const vec3f &getTgtTipRpy(const int &armId);
	const vec6f &getTgtTipVW(const int &armId);
	const vec6f &getTgtTipFM(const int &armId);

	// void setTipP(vec3f(&tipP)[2], mat3f(&tipR)[2]);
	// bool setTip(float tip[2][7]);
	bool setTip(const float tip[2][7], const float (fm[2][6])=nullptr);
	bool setTip(const vecXf tip[2], vec6f (fm[2])=nullptr);
	bool setTip(const vecNf(7)(&tip)[2]);
	bool setTip(const vecNf(7)(&tip)[2], const vec6f(&fm)[2]);
	void dwdate(float *armQ, float* armTau=nullptr);
	// void dwdate(float *armQ, float *armQd, float* armTau);
private:
	armClass();
	armClass(const armClass&)=delete;
	armClass & operator=(const armClass&)=delete;
	class impClass;
	impClass &imp;
};
}//namespace
