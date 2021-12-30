#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
using namespace std;

int main(){
    ifstream ifs;
    ifs.open("./data/shape8_at_dusk.txt");
    if (!ifs.is_open())
    {
        cout << "文件打开失败" << endl;
        return 0;
    }
    Eigen::MatrixXd data;

    //the number of magnetometer data in "./data/shape8_at_dusk.txt", change it according to your own data
    int data_nums=30280; 
    data.resize(data_nums,3);

    string line_buf;
    size_t pos=0;
    int i=0;
    while(getline(ifs,line_buf) && i< data_nums){
        pos=line_buf.find(' ');
        line_buf.erase(0,pos+1);

        pos=line_buf.find(' ');
        data(i,0)=stod(line_buf.substr(0,pos));
        line_buf.erase(0,pos+1);

        pos=line_buf.find(' ');
        data(i,1)=stod(line_buf.substr(0,pos));
        line_buf.erase(0,pos+1);

        data(i,2)= stod(line_buf);
        i++;
    }
    ifs.close();

    Eigen::MatrixXd data_x=data.col(0);
    Eigen::MatrixXd data_y=data.col(1);
    Eigen::MatrixXd data_z=data.col(2);
    Eigen::MatrixXd D_first;
    D_first.resize(data_nums,9);
    D_first.col(0)=data_x.array().square();
    D_first.col(1)=data_y.array().square();
    D_first.col(2)=data_z.array().square();
    D_first.col(3)=2*data_x.array()*data_y.array();
    D_first.col(4)=2*data_x.array()*data_z.array();
    D_first.col(5)=2*data_y.array()*data_z.array();
    D_first.col(6)=2*data_x.array();
    D_first.col(7)=2*data_y.array();
    D_first.col(8)=2*data_z.array();

    Eigen::MatrixXd D=D_first;

    Eigen::MatrixXd A_v=D.transpose()*D;
    Eigen::MatrixXd b_v=(D.transpose())*Eigen::MatrixXd::Ones(D.rows(),1);
    Eigen::MatrixXd x_v=A_v.lu().solve(b_v);

    //form the algebraic form of the ellipsoid
    Eigen::MatrixXd A;
    A.resize(4,4);
    A<<x_v(0),x_v(3),x_v(4),x_v(6),
        x_v(3),x_v(1),x_v(5),x_v(7),
        x_v(4),x_v(5),x_v(2),x_v(8),
            x_v(6),x_v(7),x_v(8),-1.0;
    //find the center of the ellipsoid
    Eigen::MatrixXd A_center=-A.block<3,3>(0,0);
    Eigen::MatrixXd b_center;
    b_center.resize(3,1);
    b_center<<x_v(6),x_v(7),x_v(8);
    Eigen::MatrixXd x_center=A_center.lu().solve(b_center);
    //form the corresponding translation matrix
    Eigen::MatrixXd T=Eigen::MatrixXd::Identity(4,4);
    T.block<1,3>(3,0)=x_center.transpose();
    //translate to the center
    Eigen::MatrixXd R=T*A*T.transpose();
    // solve the eigenproblem
    Eigen::EigenSolver<Eigen::MatrixXd> eig(R.block<3,3>(0,0)/(-R(3,3)));
    Eigen::MatrixXd eigen_vectors=eig.pseudoEigenvectors();
    Eigen::MatrixXd eigen_values=eig.pseudoEigenvalueMatrix();
    //assure the eigen values is postive
    if(eigen_values(0,0)<0){
        eigen_values(0,0)=-eigen_values(0,0);
        eigen_vectors.col(0)=-1*eigen_values.col(0);
    }
    if(eigen_values(1,1)<0){
        eigen_values(1,1)=-eigen_values(1,1);
        eigen_vectors.col(1)=-1*eigen_values.col(1);
    }
    if(eigen_values(2,2)<0){
        eigen_values(2,2)=-eigen_values(2,2);
        eigen_vectors.col(2)=-1*eigen_values.col(2);
    }
    Eigen::MatrixXd radii=(1/(eigen_values.diagonal().array())).array().sqrt();

    //get soft_corr and hard_corr
    Eigen::MatrixXd map=eigen_vectors.transpose();
    Eigen::MatrixXd inv_map=eigen_vectors;
    Eigen::MatrixXd scale_first=Eigen::MatrixXd::Identity(3,3);
    scale_first(0,0)=radii(0);scale_first(1,1)=radii(1);scale_first(2,2)=radii(2);
    Eigen::MatrixXd scale=scale_first.inverse()*radii.minCoeff();
    Eigen::MatrixXd soft_corr=inv_map*scale*map;
    Eigen::MatrixXd hard_corr=x_center;
    cout<<"soft_corr: "<<soft_corr<<endl;
    cout<<"hard_corr: "<<hard_corr<<endl;

    return 0;
}


