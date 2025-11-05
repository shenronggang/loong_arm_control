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
#define __mode_t_defined//屏蔽系统 mode_t
#include"pino.h"
#include"pinocchio/parsers/urdf.hpp"
#include"pinocchio/algorithm/kinematics.hpp"
#include"pinocchio/algorithm/jacobian.hpp"
#include"pinocchio/algorithm/rnea.hpp"
#include"pinocchio/algorithm/crba.hpp"
#include"pinocchio/algorithm/center-of-mass.hpp"
#include"iopack.h"

using namespace pinocchio;

namespace Pino{
template<typename T>
class pinoClass<T>::impClass{
public:
	ModelTpl<T> model;
	DataTpl<T> data;
	impClass(){
		// matX(T) a;
		// a.triangularView<T><Eigen::StrictlyLower>();

		// ((matX(T))(data.M)).triangularView<Eigen::StrictlyLower>() =data.M.transpose().triangularView<Eigen::StrictlyLower>();
		// data.M.triangularView<Eigen::StrictlyLower>() =data.M.transpose().triangularView<Eigen::StrictlyLower>();
	}
};
// ==================
	template<typename T>
	pinoClass<T>::pinoClass():imp(*new impClass()){}
	template<typename T>
	bool pinoClass<T>::openUrdf(const string& urdf,bool floatBase,bool verbose){
		if(access(urdf.data(), F_OK) ==-1){
			throw runtime_error(urdf+" 不存在！");
			return 0;
		}
		Model modelDouble;
		if(floatBase){
			JointModelFreeFlyer flyJoint;
			pinocchio::urdf::buildModel(urdf,flyJoint,modelDouble,verbose);
		}else{
			pinocchio::urdf::buildModel(urdf,modelDouble,verbose);
		}
		imp.model=modelDouble.cast<T>();
		imp.data=DataTpl<T>(imp.model);
		return 1;
	}
	template<typename T>
	void* pinoClass<T>::getModel(){
		return &imp.model;
	}
	template<typename T>
	void* pinoClass<T>::getData(){
		return &imp.data;
	}
	template<typename T>
	int pinoClass<T>::getDof(){
		return imp.model.nv;
	}
	template<typename T>
	int pinoClass<T>::getJntId(const string& jntName){
		return imp.model.getJointId(jntName);
	}
	template<typename T>
	T pinoClass<T>::getTotalMass(){
		return computeTotalMass(imp.model, imp.data);
	}





	template<typename T>
	void pinoClass<T>::calKin(vecX(T) &q, vecX(T) &qd){
		forwardKinematics(imp.model, imp.data, q, qd);
	}
	template<typename T>
	void pinoClass<T>::calKin(vecX(T) &q){
		forwardKinematics(imp.model, imp.data, q);
	}
	template<typename T>
	void pinoClass<T>::calJcb(vecX(T) &q){
		computeJointJacobians(imp.model, imp.data, q);
	}
	template<typename T>
	void pinoClass<T>::calJcbDot(vecX(T) &q, vecX(T) &qd){
		computeJointJacobiansTimeVariation(imp.model, imp.data, q,qd);
	}
	template<typename T>
	void pinoClass<T>::crba(vecX(T) &q){
		pinocchio::crba(imp.model, imp.data, q);//存储于data.M上三角
		// (imp.data.M).triangularView<Eigen::StrictlyLower>() =(imp.data.M).transpose().triangularView<Eigen::StrictlyLower>();
	}
	template<typename T>
	void pinoClass<T>::noneLinear(vecX(T) &q, vecX(T) &qd){
		nonLinearEffects(imp.model, imp.data, q,qd);//存储于data.nle
	}





	template<typename T>
	const vec3(T)& pinoClass<T>::getP(const int &jntId){
		return imp.data.oMi[jntId].translation();
	}
	template<typename T>
	const mat3(T)& pinoClass<T>::getR(const int &jntId){
		return imp.data.oMi[jntId].rotation();
	}
	template<typename T>
	vec3(T) pinoClass<T>::getV(const int &jntId){
		return imp.data.ov[1].linear()+imp.data.ov[1].angular().cross(imp.data.oMi[1].translation());
	}
	template<typename T>
	vec3(T) pinoClass<T>::getW(const int &jntId){
		return imp.data.ov[jntId].angular();
	}
	template<typename T>
	void pinoClass<T>::getVW(const int &jntId, vec6(T) &vw){
		vw<<imp.data.ov[1].linear()+imp.data.ov[1].angular().cross(imp.data.oMi[1].translation()),
			imp.data.ov[jntId].angular();
	}
	template<typename T>
	void pinoClass<T>::getJcb(const int &jntId, matX(T)& J){
		getJointJacobian(imp.model,imp.data,jntId,LOCAL_WORLD_ALIGNED,J);
	}
	template<typename T>
	void pinoClass<T>::getJcbDot(const int &jntId, matX(T)& Jd){
		getJointJacobianTimeVariation(imp.model,imp.data,jntId,LOCAL_WORLD_ALIGNED,Jd);
	}
	template<typename T>
	const matX(T)& pinoClass<T>::getM(){
		return imp.data.M;
	}
	template<typename T>
	const vecX(T)& pinoClass<T>::getNoneLinear(){
		return imp.data.nle;
	}


template class pinoClass<float>;
template class pinoClass<double>;
}//namespace
