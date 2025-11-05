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
pinocchino每次编译太笨重了，非常耗时，因此做一层封装
=====================================================*/
#pragma once
#include"eigen.h"


namespace Pino{
template<typename T>
class pinoClass{
public:
	pinoClass();
	// bool openUrdf(const string& urdf);
	bool openUrdf(const string& urdf,bool floatBase=0,bool verbose=0);
	void* getModel();//返回pino model指针
	void* getData(); //返回pino data指针
	int getDof();
	int getJntId(const string& jntName); //关节id
	T getTotalMass();

	void calKin(vecX(T) &q);
	void calKin(vecX(T) &q, vecX(T) &qd);
	void calJcb(vecX(T) &q);
	void calJcbDot(vecX(T) &q, vecX(T) &qd);
	void crba(vecX(T) &q);//得到M
	void noneLinear(vecX(T) &q, vecX(T) &qd);//得到h
	void calCom();
	void calComJcb();

	const vec3(T)& getP(const int &jntId);
	const mat3(T)& getR(const int &jntId);
	vec3(T) getV(const int &jntId);//不能const引用
	vec3(T) getW(const int &jntId);//不能const引用
	void getVW(const int &jntId, vec6(T) &vw);
	void getJcb(const int &jntId, matX(T) &J);
	void getJcbDot(const int &jntId, matX(T) &Jd);
	const matX(T) &getM();
	const vecX(T) &getNoneLinear();
private:
	class impClass;
	impClass &imp;
};
}//namespace
