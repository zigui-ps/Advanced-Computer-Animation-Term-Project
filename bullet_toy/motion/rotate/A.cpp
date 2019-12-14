#include<Eigen/Dense>
#include<stdio.h>

using namespace Eigen;

int main()
{
	const double PI = acos(-1);
	char buf[1000000];
	while(fgets(buf, 1000000, stdin)){
		printf("%s", buf);
		if(strcmp(buf, "MOTION\n") == 0) break;
	}
	int n;
	scanf("Frames: %d\n", &n); printf("Frames: %d\n", n); 
	fgets(buf, 1000000, stdin); printf("%s", buf);
	for(int i = 1; i <= n; i++){
		double ox, oy, oz, z, x, y;
		scanf("%lf%lf%lf%lf%lf%lf", &ox, &oy, &oz, &z, &y, &x);
		Affine3d m = Affine3d::Identity();
		m.rotate(AngleAxisd(z / 180 * PI, Vector3d::UnitZ()));
		m.rotate(AngleAxisd(y / 180 * PI, Vector3d::UnitY()));
		m.rotate(AngleAxisd(x / 180 * PI, Vector3d::UnitX()));
		m.pretranslate(Vector3d(ox, oy, oz));
		//m.prerotate(AngleAxisd(0.5 * PI, Vector3d::UnitX()));
		fprintf(stderr, "%lf %lf %lf %lf %lf %f\n", ox, oy, oz, z, x, y);

		Vector3d t = m.translation();
		printf("%lf %lf %lf ", t[0], t[1], t[2]);
		Vector3d ea = m.linear().eulerAngles(2, 1, 0) * (180 / PI);
		printf("%lf %lf %lf ", ea[0], ea[1], ea[2]);
		fprintf(stderr, "%lf %lf %lf %lf %lf %f\n", t[0], t[1], t[2], ea[0], ea[1], ea[2]);
		fgets(buf, 1000000, stdin); printf("%s", buf);
	}
}
