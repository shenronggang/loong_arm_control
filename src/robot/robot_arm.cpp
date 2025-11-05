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
浮基6 +臂7*2 +头2 +腰3 +腿6*2
0     6      20  22   25   37
pino本身的的q是W系而qd、qdd、tau都是在B系，关系到雅可比、惯量等，因此使用时qd、qdd的【前6维】需要做变换
v2W=J*w2B，故qd∈B
t2B=JT*f2W，故tau∈B
=====================================================*/
#include"robot_arm.h"
#include"pino.h"
// #include"pino_float.h"
#include"iopack.h"
#include"eigen.h"
#include"pInv.h"
#include"geom_sol/wrist_ofst_7dof/ik_7dof_ofst.h"
#include"geom_sol/wrist_ofst_7dof/fk_with_ofst.h"

namespace Rbt{
class armClass::impClass{
public:
	impClass(armClass*omp);
	armClass &omp;
	Pino::pinoClass<float> pino;
	// Pino::pinoFloatClass pino;

	vecXf q,qd,tau;
	vecXf tgtQ,tgtQd,tgtTau;
	int shoulderId[2],wristId[2],elbowId[2];
	int qdMemSize,tipMemSize;
	matXf JcbQ,Jcb[2];
	vec3f shoulderP[2];
	vec3f actTipP[2],cmdTipP[2],actElbowP[2],cmdElbowP[2];
	mat3f actTipR[2],cmdTipR[2];
	mat3f tipRAdj[2];//urdf腕坐标转到B
	vec3f actTipRpy[2],cmdTipRpy[2];
	vec6f actTipVW[2],cmdTipVW[2];
	vec6f actTipFM[2],cmdTipFM[2];
	float cmdArmAng[2];//臂型角
	float l1,l2,lmax;
	double curLeftQ[NArmDof] = {0.0};
	double curRightQ[NArmDof] = {0.0};
	double lastLeftQ[NArmDof] = {0.0};
	double lastRightQ[NArmDof] = {0.0};
	int ikStaLeft = 0;
	int ikStaRight = 0;
	double leftExactTheta[NArmDof] = {0.0};
	double rightExactTheta[NArmDof] = {0.0};
	int lOrRleft = 1;
	int fOrBleft = 1;
	int lOrRight = 1;
	int fOrBight = 1;


	// void update(const float(&jj)[NArmDof]);
	void openUrdf(const string &urdf);
	void update(float *armQ, float *armQd, float* armTau);
	bool setTip(const float tip[2][7], const float ff[2][6]);
	bool setTip(const vecNf(7)(&tip)[2], const vec6f(&fm)[2]);
	bool checkWorkspace(vec3f (&cmdP)[2]);
	bool p2j();
	void f2t();
	bool geomP2j();
};
	armClass::impClass::impClass(armClass*omp):omp(*omp){
		tipRAdj[0]<<0,0,-1,  1,0,0,  0,-1,0;
		tipRAdj[1]<<0,0,-1,  -1,0,0,  0,1,0;
		q.setZero(NArmDof*2);
		qd.setZero(NArmDof*2);
		tau.setZero(NArmDof*2);
		tgtQ.setZero(NArmDof*2);
		tgtQd.setZero(NArmDof*2);
		tgtTau.setZero(NArmDof*2);
		JcbQ.setZero(6,NArmDof*2);
		Jcb[0].setZero(6,NArmDof);
		Jcb[1].setZero(6,NArmDof);
		openUrdf("../model/AzureLoong_v2.5_arms.urdf");
	}
	void armClass::impClass::openUrdf(const string &urdf){
		pino.openUrdf(urdf);

		shoulderId[0]=pino.getJntId("J_arm_l_02");
		shoulderId[1]=pino.getJntId("J_arm_r_02");
		wristId[0]=pino.getJntId("J_arm_l_07");
		wristId[1]=pino.getJntId("J_arm_r_07");
		elbowId[0]=pino.getJntId("J_arm_l_04");
		elbowId[1]=pino.getJntId("J_arm_r_04");

		pino.calKin(q,qd);
		shoulderP[0]=pino.getP(shoulderId[0]);
		shoulderP[1]=pino.getP(shoulderId[1]);
		l1=(pino.getP(elbowId[0]) -shoulderP[0]).norm();
		l2=(pino.getP(elbowId[0]) -pino.getP(wristId[0])).norm();
		lmax=l1+l2-0.01;

		qdMemSize=pino.getDof()*4;
		tipMemSize=6*4;
		// print(model.nq ,shoulderId[0],shoulderId[1], wristId[0], wristId[1]);
	}
	bool armClass::impClass::checkWorkspace(vec3f (&cmdP)[2]){
		vec3f tmp;
		bool overLen=0;
		For2{
			tmp=cmdP[i]-shoulderP[i];
			float l=tmp.norm();
			if(l>lmax){
				tmp*=lmax/l;
				overLen=1;
			}
			cmdP[i]=shoulderP[i]+tmp;
		}
		return overLen;
	}
	// void armClass::impClass::update(const float(&jj)[NGenDof]){
	// 	memcpy(q.data(), jj, MemDof);
	// 	forwardKinematics(model,data,q);
	// 	// omp.p[0]=pino.getP(ankId[0]);
	// 	// omp.p[1]=pino.getP(ankId[1]);
	// 	// omp.p[2]=pino.getP(wristId[0]);
	// 	// omp.p[3]=pino.getP(wristId[1]);

	// 	// omp.R[0]=pino.getP(ankId[0]);
	// 	// omp.R[1]=pino.getP(ankId[1]);
	// 	// omp.R[2]=pino.getP(wristId[0]);
	// 	// omp.R[3]=pino.getP(wristId[1]);
	// }
	
	void armClass::impClass::update(float *armQ, float *armQd, float* armTau){
		memcpy(q.data(), armQ, qdMemSize);
		memcpy(qd.data(), armQd, qdMemSize);
		memcpy(tau.data(), armTau, qdMemSize);
		pino.calKin(q,qd);

		actTipP[0]=pino.getP(wristId[0]);
		actTipP[1]=pino.getP(wristId[1]);
		actTipR[0]=pino.getR(wristId[0]) *tipRAdj[0];
		actTipR[1]=pino.getR(wristId[1]) *tipRAdj[1];
		// actTipR[0]=pino.getR(wristId[0]);
		// actTipR[1]=pino.getR(wristId[1]);

		Ei::R2rpy(actTipR[0],actTipRpy[0]);
		Ei::R2rpy(actTipR[1],actTipRpy[1]);

		pino.getVW(wristId[0], actTipVW[0]);
		pino.getVW(wristId[1], actTipVW[1]);

		// Ei::pInv3()
		pino.calJcb(q);
		pino.getJcb(wristId[0], JcbQ);
		Jcb[0]=JcbQ.leftCols<NArmDof>();
		pino.getJcb(wristId[1], JcbQ);
		Jcb[1]=JcbQ.rightCols<NArmDof>();

		actTipFM[0]=Jcb[0].transpose().householderQr().solve(tau.segment<NArmDof>(0));
		actTipFM[1]=Jcb[1].transpose().householderQr().solve(tau.segment<NArmDof>(NArmDof));
	}
	bool armClass::impClass::p2j(){
		checkWorkspace(cmdTipP);
		// printEi("cmd",cmdTipP[0]);
		// printEi(cmdTipP[1]);
		// printEi("肩",shoulderP[0]);
		// printEi(shoulderP[1]);

		tgtQ=q;
		//==几何求肘位置==
		vecNf(NArmDof) nearJ[2];
		For2{
			int sign=1-i*2;//左1右-1
			//===先假设肘竖直面
			vec3f tmpP=cmdTipP[i]-shoulderP[i];
			// printEi("tmpP",tmpP);
			float l=tmpP.norm();
			float lxy=hypot(tmpP[0],tmpP[1]);
			float ang1=acos((l1*l1 +l*l -l2*l2)/(2*l1*l));
			//===竖直面求肘位置
			cmdElbowP[i]=tmpP/l*l1;
			vec3f tmpN;
			tmpN<<-tmpP[1],tmpP[0],0;
			//=手腕x在肩下附近时，臂型角会显得不合理（近奇异突变、肘碰身体），故修改臂型角定义（即竖直面和平行x面加权），使预处理肘为趋近向身后
			float rateX=5*abs(tmpP[0]);
			if(rateX<1){
				tmpN=rateX*tmpN +(1-rateX)*vec3f(0,-tmpP[2],tmpP[1]);
			}
			tmpN.normalize();
			cmdElbowP[i]=aAxif(ang1,tmpN)*cmdElbowP[i];
			//臂型角旋转
			cmdElbowP[i]=aAxif(sign*cmdArmAng[i], tmpP.normalized())*cmdElbowP[i];
			//===粗略逆解（肩肘腕不共线，仅做范围约束，防止多解）
			nearJ[i][3]=Pi-acos((l1*l1 +l2*l2 -l*l)/(2*l1*l2));
			nearJ[i][0]=-sign*atan2(cmdElbowP[i][0],-cmdElbowP[i][2]);
			nearJ[i][1]=-acos(sign*cmdElbowP[i][1]/l1);
			//=转到大臂坐标，求大臂自转
			mat3f tmpR=Ei::Ry(sign*nearJ[i][0]) *Ei::Rx<float>(sign*(P2i+nearJ[i][1]));
			tmpP=tmpR.transpose() *(tmpP-cmdElbowP[i]);
			// printEi("tmpP",tmpP);
			nearJ[i][2]=atan2(sign*tmpP[0],sign*tmpP[1]);
			cmdElbowP[i]+=shoulderP[i];
		}
		tgtQ.segment<4>(0)=nearJ[0].head<4>();
		tgtQ.segment<4>(NArmDof)=nearJ[1].head<4>();
		pino.calKin(tgtQ,qd);
		//elbowR *R456 =cmdTipR
		// std::cout << "cmdTipR: " << cmdTipR[0] << std::endl;
		mat3f R456[2]{pino.getR(elbowId[0]).transpose()*cmdTipR[0],
					  pino.getR(elbowId[1]).transpose()*cmdTipR[1]};
		// 左Ry*R(-x)*Rz， 右R(-y)*Rx*Rz
		// Ry*Rx*Rz=			cx*sy;
		//			cx*sz, cx*cz, -sx;
		//						cx*cy;
		nearJ[0][4]=atan2(R456[0](0,2), R456[0](2,2));
		nearJ[0][5]=-asin(R456[0](1,2));
		nearJ[0][6]=atan2(R456[0](1,0), R456[0](1,1));

		nearJ[1][4]=-atan2(R456[0](0,2), R456[0](2,2));
		nearJ[1][5]=asin(R456[0](1,2));
		nearJ[1][6]=atan2(R456[0](1,0), R456[0](1,1));

		tgtQ<<nearJ[0],nearJ[1];

		// printEi("肘",cmdElbowP[0]);
		// printEi(cmdElbowP[1]);
		// printEi("nearJ",nearJ[0]);
		// printEi(nearJ[1]);

		static const int taskRows=9;
		float step=0.08,rate=0.6,eps=0.001;
		vecNf(taskRows) err[2];
		vecNf(NArmDof) dq[2];
		matXf Jwrist[2],Jelbow[2];
		Jwrist[0].setZero(6,NArmDof*2);
		Jwrist[1].setZero(6,NArmDof*2);
		Jelbow[0].setZero(6,NArmDof*2);
		Jelbow[1].setZero(6,NArmDof*2);
		matNf(taskRows, NArmDof) J[2];
		matNf(NArmDof,NArmDof) JtJ[2];
		Jwrist[0].setZero();Jwrist[1].setZero();
		Jelbow[0].setZero();Jelbow[1].setZero();
		J[0].setZero();J[1].setZero();

		aAxif tmpA[2];
		int kk=0;

		for(int k=0;k<500;k++){
			pino.calJcb(tgtQ);
			For2{
				tmpA[i]=cmdTipR[i]*pino.getR(wristId[i]).transpose();
				err[i]<<cmdTipP[i] -pino.getP(wristId[i]),
						tmpA[i].angle()*tmpA[i].axis(),
						cmdElbowP[i] -pino.getP(elbowId[i]);
						// nearJ0[i]-tgtQ[i*NArmDof],
						// nearJ1[i]-tgtQ[i*NArmDof+1],
						// nearJ2[i]-tgtQ[i*NArmDof+2],
						// nearJ3[i]-tgtQ[i*NArmDof+3];
				pino.getJcb(wristId[i],Jwrist[i]);
				pino.getJcb(elbowId[i],Jelbow[i]);
				
				J[i].topRows<6>()=Jwrist[i].block<6,NArmDof>(0,NArmDof*i);
				J[i].middleRows<3>(6)=Jelbow[i].block<3,NArmDof>(0,NArmDof*i);

				// // 肩肘腕不共线，仅做范围约束，防止多解
				// if(err[i].tail<4>().norm()>0.5){
				// 	J[i](taskRows-4, 0)=1;
				// 	J[i](taskRows-3, 1)=1;
				// 	J[i](taskRows-2, 2)=1;
				// 	J[i](taskRows-1, 3)=1;
				// }else{
				// 	J[i](taskRows-4, 0)=0;
				// 	J[i](taskRows-3, 1)=0;
				// 	J[i](taskRows-2, 2)=0;
				// 	J[i](taskRows-1, 3)=0;
				// 	err[i].tail<3>().setZero();
				// }

				for(int j=0;j<taskRows;j++){
					Alg::clip(err[i][j], step);
				}

				JtJ[i]=J[i].transpose()*J[i];
				JtJ[i].diagonal().array()+=1e-6;
				dq[i]=rate* JtJ[i].householderQr().solve(J[i].transpose()*err[i]);
				// dq[i][3]=0;
				tgtQ.segment<NArmDof>(i*NArmDof)+=dq[i];
				kk=k;
			}
			// if(k>200){print(k);}

			// fprint(k);
			// fprintEi("err",err[0]);
			// fprintEi(err[1]);
			// fprintEi("dq",dq[0]);
			// fprintEi(dq[1]);
			// fprintEi("tgtQ",tgtQ.head<NArmDof>());
			// fprintEi(tgtQ.tail<NArmDof>());

			if(dq[0].norm()<eps && dq[1].norm()<eps){
				break;
			}
		}
// print(kk);
		// printEi("肘",pino.getP(elbowId[0]));
		// printEi(pino.getP(elbowId[1]));
		// printEi("手",pino.getP(wristId[0]));
		// printEi(pino.getP(wristId[1]));
		// vec3f rpy[2];
		// rpy[0]=Ei::R2rpy<float>(pino.getR(wristId[0]) *tipRAdj[0]);
		// rpy[1]=Ei::R2rpy<float>(pino.getR(wristId[1]) *tipRAdj[1]);
		// printEi("手rpy",rpy[0]);
		// printEi(rpy[1]);

		

		return 0;
	}
	void armClass::impClass::f2t(){
		tgtTau.head<NArmDof>()=Jcb[0].transpose()*cmdTipFM[0];
		tgtTau.tail<NArmDof>()=Jcb[1].transpose()*cmdTipFM[1];
	}
//=========================================
	armClass& armClass::instance(){
		static armClass singtn;
		return singtn;
	}
	armClass::armClass():imp(*new impClass(this)){}
	void armClass::setDt(float dt){}
	// void update(vecNf(NArmMot)&armQ, vecNf(NArmMot)&armQd, vecNf(NArmMot)&armTau){}
	void armClass::update(float *armQ, float *armQd, float* armTau){
		imp.update(armQ,armQd,armTau);
	}
	const vec3f &armClass::getTipP(const int &armId){return imp.actTipP[armId];}
	const mat3f &armClass::getTipR(const int &armId){return imp.actTipR[armId];}
	const vec3f &armClass::getTipRpy(const int &armId){return imp.actTipRpy[armId];}
	const vec6f &armClass::getTipVW(const int &armId){return imp.actTipVW[armId];}
	const vec6f &armClass::getTipFM(const int &armId){return imp.actTipFM[armId];}

	const vec3f &armClass::getTgtTipP(const int &armId){return imp.cmdTipP[armId];}
	// const mat3f &armClass::getTgtTipR(const int &armId){return imp.cmdTipR[armId];}
	const vec3f &armClass::getTgtTipRpy(const int &armId){return imp.cmdTipRpy[armId];}
	const vec6f &armClass::getTgtTipVW(const int &armId){return imp.cmdTipVW[armId];}
	const vec6f &armClass::getTgtTipFM(const int &armId){return imp.cmdTipFM[armId];}


	bool armClass::setTip(const float tip[2][7], const float fm[2][6]){
		imp.cmdTipP[0]<<tip[0][0], tip[0][1], tip[0][2];
		imp.cmdTipP[1]<<tip[1][0], tip[1][1], tip[1][2];
		imp.cmdTipRpy[0]<<tip[0][3], tip[0][4], tip[0][5];
		imp.cmdTipRpy[1]<<tip[1][3], tip[1][4], tip[1][5];
		imp.cmdTipR[0]=Ei::rpy2R(tip[0][3], tip[0][4], tip[0][5]) *imp.tipRAdj[0].transpose();
		imp.cmdTipR[1]=Ei::rpy2R(tip[1][3], tip[1][4], tip[1][5]) *imp.tipRAdj[1].transpose();
		imp.cmdArmAng[0]=tip[0][6];
		imp.cmdArmAng[1]=tip[1][6];
		if(fm==nullptr){
			imp.cmdTipFM[0].setZero();
			imp.cmdTipFM[1].setZero();
		}else For6{
			imp.cmdTipFM[0][i]=fm[0][i];
			imp.cmdTipFM[1][i]=fm[1][i];
		}
		imp.f2t();
		return imp.p2j();
	}
	bool armClass::setTip(const vecXf tip[2],vec6f fm[2]){
		imp.cmdTipP[0]=tip[0].head<3>();
		imp.cmdTipP[1]=tip[1].head<3>();

		imp.cmdTipRpy[0]<<tip[0].segment<3>(3);
		imp.cmdTipRpy[1]<<tip[1].segment<3>(3);

		// imp.cmdTipR[0]=Ei::rpy2R(tip[0][3], tip[0][4], tip[0][5]) *imp.tipRAdj[0].transpose();
		// imp.cmdTipR[1]=Ei::rpy2R(tip[1][3], tip[1][4], tip[1][5]) *imp.tipRAdj[1].transpose();
		imp.cmdTipR[0]=Ei::rpy2R(tip[0][3], tip[0][4], tip[0][5]);
		imp.cmdTipR[1]=Ei::rpy2R(tip[1][3], tip[1][4], tip[1][5]);
		
		imp.cmdArmAng[0]=tip[0][6];
		imp.cmdArmAng[1]=tip[1][6];
		imp.cmdTipFM[0]=fm[0];
		imp.cmdTipFM[1]=fm[1];
		imp.f2t();
		// return imp.p2j();
		return imp.geomP2j();
	};
	bool armClass::setTip(const vecNf(7)(&tip)[2]){
		imp.cmdTipP[0]=tip[0].head<3>();
		imp.cmdTipP[1]=tip[1].head<3>();

		imp.cmdTipRpy[0]<<tip[0].segment<3>(3);
		imp.cmdTipRpy[1]<<tip[1].segment<3>(3);
		imp.cmdTipR[0]=Ei::rpy2R(tip[0][3], tip[0][4], tip[0][5]) *imp.tipRAdj[0].transpose();
		imp.cmdTipR[1]=Ei::rpy2R(tip[1][3], tip[1][4], tip[1][5]) *imp.tipRAdj[1].transpose();
		imp.cmdArmAng[0]=tip[0][6];
		imp.cmdArmAng[1]=tip[1][6];
		imp.cmdTipFM[0].setZero();
		imp.cmdTipFM[1].setZero();
		imp.f2t();
		return imp.p2j();
	}
	bool armClass::setTip(const vecNf(7)(&tip)[2], const vec6f(&fm)[2]){
		imp.cmdTipP[0]=tip[0].head<3>();
		imp.cmdTipP[1]=tip[1].head<3>();

		imp.cmdTipRpy[0]<<tip[0].segment<3>(3);
		imp.cmdTipRpy[1]<<tip[1].segment<3>(3);
		imp.cmdTipR[0]=Ei::rpy2R(tip[0][3], tip[0][4], tip[0][5]) *imp.tipRAdj[0].transpose();
		imp.cmdTipR[1]=Ei::rpy2R(tip[1][3], tip[1][4], tip[1][5]) *imp.tipRAdj[1].transpose();
		imp.cmdArmAng[0]=tip[0][6];
		imp.cmdArmAng[1]=tip[1][6];
		imp.cmdTipFM[0]=fm[0];
		imp.cmdTipFM[1]=fm[1];
		imp.f2t();
		return imp.p2j();
	}
	// bool armClass::setTip(float tip[2][7]){return imp.setTip(tip,nullptr);}
	void armClass::dwdate(float *armQ, float* armTau){
		For(NArmDof*2){
			armQ[i]=imp.tgtQ[i];
		}
		if(armTau!=nullptr){
			For(NArmDof*2){
				armTau[i]=imp.tgtTau[i];
			}
		}
	}
	bool armClass::impClass::geomP2j(){
		checkWorkspace(cmdTipP);

		if (ikStaLeft == 0){
			For(NArmDof){
				curLeftQ[i] = q[i];
			}
		}
		if (ikStaRight == 0){
			For(NArmDof){
				curRightQ[i] = q[i+NArmDof];
			}
		}
		
		// std::cout << "curLeftQ: " << curLeftQ[0] << " " << curLeftQ[1] << " " << curLeftQ[2] << " " << curLeftQ[3] << " " << curLeftQ[4] << " " << curLeftQ[5] << " " << curLeftQ[6] << std::endl;
		// std::cout << "curRightQ: " << curRightQ[0] << " " << curRightQ[1] << " " << curRightQ[2] << " " << curRightQ[3] << " " << curRightQ[4] << " " << curRightQ[5] << " " << curRightQ[6] << std::endl;

		// ik
		double jntLim[NMotArm] = {-3.1416, 3.1416, -2.6180, 2.6180, -3.1416, 3.1416, -0.8727, 2.8798, -3.1416, 3.1416, -1.5708, 1.5708, -1.5708, 1.5708};
		double a[3] = {24.0, 24.0, 54.0};
		double d[2] = {300.0, 246.0};
		double leftTheta[NArmDof] = {0.0};
		double rightTheta[NArmDof] = {0.0};
		double leftBet = cmdArmAng[0];
		double rightBet = -cmdArmAng[1];
		int bLOrR = 0;
		vec3f pInW[2], pInShld[2], rotVec[2];
		mat3f RfrmShldr2w[2], rotMat[2], Rini0[2];

		// fk
		double aArr[NArmDof] = {0, 0, 0, a[0], -a[1], 0, -a[2]};
		double alphaArr[NArmDof] = {0, -Pi/2, Pi/2, -Pi/2, Pi/2, Pi/2, -Pi/2};
		double dArr[NArmDof] = {0, 0, d[0], 0, d[1], 0, 0};
		double thetaArr[NArmDof] = {0, 0, 0, 0, 0, -Pi/2, 0};
		double leftCarteCalc[3] = {0.0};
		double rightCarteCalc[3] = {0.0};
		double leftEulCalc[3] = {0.0};
		double rightEulCalc[3] = {0.0};
		double leftBetCalc = 0.0;
		double rightBetCalc = 0.0;
		int lOrRCalc = 0;
		int fOrBCalc = 0;

		// 左臂
		bLOrR = 0; // 0-左臂
		pInW[0] = cmdTipP[0]-shoulderP[0];
		RfrmShldr2w[0] << 0,0,1,  1,0,0,  0,1,0;  // 肩部坐标系转到中心坐标系
		pInShld[0] = RfrmShldr2w[0] * pInW[0];
		Rini0[0] << -1,0,0,  0,0,-1,  0,-1,0;  // 初始位置手掌朝下
		rotMat[0] = RfrmShldr2w[0]*cmdTipR[0]*Rini0[0];
		Ei::R2rpy(rotMat[0], rotVec[0]);
		double px = double(pInShld[0](0)*1000);
		double py = double(pInShld[0](1)*1000);
		double pz = double(pInShld[0](2)*1000);
		double rotz = double(rotVec[0](2));
		double roty = double(rotVec[0](1));
		double rotx = double(rotVec[0](0));
		ik_7dof_ofst(rotz, roty, rotx, px, py, pz, leftBet, curLeftQ, bLOrR, lOrRleft, fOrBleft, jntLim, a, d, leftTheta, &ikStaLeft);
		double leftArmLen = sqrt(px*px+py*py+pz*pz);
		if (leftArmLen > 590){
			printL("警告-左臂接近伸直");;
		}
		

		double leftThetaMid[NArmDof] = {0.0};
		for (int i = 0; i < NArmDof; i++){
			leftThetaMid[i] = leftTheta[i];
		}
		fk_with_ofst(leftThetaMid, bLOrR, aArr, alphaArr, dArr, thetaArr, leftCarteCalc, leftEulCalc, &leftBetCalc, &lOrRCalc, &fOrBCalc);

		
		if ((leftBetCalc < (leftBet+0.01)) && leftBetCalc > (leftBet-0.01)){
			lOrRleft = 1;
			fOrBleft = 1;
			for (int i = 0; i < NArmDof; i++){
				leftExactTheta[i] = leftTheta[i];
			}
		}
		else{
			lOrRleft = 1;
			fOrBleft = 0;
			for (int i = 0; i < NArmDof; i++){
				leftExactTheta[i] = leftTheta[i];
				if (leftExactTheta[4] > Pi){
					leftExactTheta[4] = leftExactTheta[4]-2*Pi;
					if (ikStaLeft == 2041){
						ikStaLeft = 0;
					}
				}
			}
		}
		// std::cout << "ikStaLeft: " << ikStaLeft << std::endl;
		
		if (ikStaLeft != 0){
			printL("左臂有问题，错误编号-", ikStaLeft);
		}

		// 右臂
		bLOrR = 1; // 1-右臂
		pInW[1] = cmdTipP[1]-shoulderP[1];
		RfrmShldr2w[1] << 0,0,1,  -1,0,0,  0,-1,0;  // 肩部坐标系转到中心坐标系
		pInShld[1] = RfrmShldr2w[1] * pInW[1];
		Rini0[1] << -1,0,0,  0,0,1,  0,1,0;  // 初始位置手掌朝下
		rotMat[1] = RfrmShldr2w[1]*cmdTipR[1]*Rini0[1];
		Ei::R2rpy(rotMat[1], rotVec[1]);
		px = double(pInShld[1](0)*1000);
		py = double(pInShld[1](1)*1000);
		pz = double(pInShld[1](2)*1000);
		rotz = double(rotVec[1](2));
		roty = double(rotVec[1](1));
		rotx = double(rotVec[1](0));
		ik_7dof_ofst(rotz, roty, rotx, px, py, pz, rightBet, curRightQ, bLOrR, lOrRight, fOrBight, jntLim, a, d, rightTheta, &ikStaRight);
		double rightArmLen = sqrt(px*px+py*py+pz*pz);
		if (rightArmLen > 590){
			printL("警告-右臂接近伸直");;
		}
		
		double rightThetaMid[NArmDof] = {0.0};
		for (int i = 0; i < NArmDof; i++){
			rightThetaMid[i] = rightTheta[i];
		}
		fk_with_ofst(rightThetaMid, bLOrR, aArr, alphaArr, dArr, thetaArr, rightCarteCalc, rightEulCalc, &rightBetCalc, &lOrRCalc, &fOrBCalc);

		if ((rightBetCalc < (rightBet+0.01)) && rightBetCalc > (rightBet-0.01)){
			lOrRight = 1;
			fOrBight = 1;
			for (int i = 0; i < NArmDof; i++){
				rightExactTheta[i] = rightTheta[i];
			}
		}
		else{
			lOrRight = 1;
			fOrBight = 0;
			for (int i = 0; i < NArmDof; i++){
				rightExactTheta[i] = rightTheta[i];
				if (rightExactTheta[4] > Pi){
					rightExactTheta[4] = rightExactTheta[4]-2*Pi;
					if (ikStaRight == 2141){
						ikStaRight = 0;
					}
				}
			}
		}

		// std::cout << "ikStaLeft: " << ikStaLeft << std::endl;
		if (ikStaRight != 0){
			printL("右臂有问题，错误编号-", ikStaRight);
		}

		vecNf(NArmDof) exactJ[2];
		For(NArmDof){
			exactJ[0][i] = float(leftExactTheta[i]);
			exactJ[1][i] = float(rightExactTheta[i]);
		}
		tgtQ << exactJ[0],exactJ[1];
		return 0;
	}
}//namespace






