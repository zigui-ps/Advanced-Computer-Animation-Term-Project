#include<Eigen/Dense>
#include "Skeleton/Skeleton.h"

IKConstraint::IKConstraint(SkeletonNodePtr node, Eigen::Affine3d target): node(node), target(target){}

void Skeleton::inverseKinematics(const std::vector<IKConstraint> &constraint)
{
	if(constraint.empty()) return;
	auto apply = [&](Eigen::VectorXd x){
		location += x.segment(0, 3);
		for(int i = 0; i < nodeList.size(); i++){
			Eigen::Vector3d rotv = x.segment((i+1)*3, 3);
			if(i) rotv = nodeLocation[nodeList[i]->parent->idx].linear().inverse() * rotv;
			Eigen::Quaterniond rot = Eigen::Quaterniond(Eigen::AngleAxisd(rotv.norm(), rotv.normalized()));
			nodeList[i]->jointTransform = rot * nodeList[i]->jointTransform;
		}
	};

	auto getb = [&](){
		Eigen::VectorXd b(6 * constraint.size());
		for(int i = 0; i < constraint.size(); i++){
			Eigen::Affine3d target = constraint[i].target;
			Eigen::Affine3d current = nodeLocation[constraint[i].node->idx];

			Eigen::AngleAxisd tmp = Eigen::AngleAxisd(target.linear() * current.linear().inverse());
			b.segment(i*6, 3) = target.translation() - current.translation();
			b.segment(i*6+3, 3) = tmp.angle() * tmp.axis();
		}
		return b;
	};

	int t;
	double lst = 1e9;
	for(t = 0; t < 40; t++){
		forwardKinematics();

		// get b
		Eigen::VectorXd b = getb();
		double dist = b.norm();
		if(dist < 1e-6) break;
		lst = dist;

		// get Jacobian
		Eigen::MatrixXd jacobian = getJacobian(constraint);

		// solve 
		Eigen::VectorXd d = jacobian.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b).normalized();

		// calculate delta
		double sigma = 1e-6;
		double df0 = 2*b.transpose()*jacobian*d;

		apply(d * sigma);
		Eigen::MatrixXd j = getJacobian(constraint);
		double dfs = (2*getb().transpose() * j * d).value();
		double delta = std::min(1., -sigma * df0 / (dfs - df0));

		apply(d * (delta-sigma));
	}
}
