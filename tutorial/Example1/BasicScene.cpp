#include "BasicScene.h"
#include <read_triangle_mesh.h>
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include "igl/read_triangle_mesh.cpp"
#include "igl/edge_flaps.h"
#include "igl/AABB.h"
#include <utility>

// #include "AutoMorphingModel.h"

using namespace cg3d;

void BasicScene::RenderBoundingBox(Eigen::MatrixXd V,Eigen::MatrixXi F){
// Find the bounding box
    Eigen::Vector3d m = V.colwise().minCoeff();
    Eigen::Vector3d M = V.colwise().maxCoeff();


    // Corners of the bounding box
    Eigen::MatrixXd V_box(8,3);
    V_box <<
          m(0), m(1), m(2),
            M(0), m(1), m(2),
            M(0), M(1), m(2),
            m(0), M(1), m(2),
            m(0), m(1), M(2),
            M(0), m(1), M(2),
            M(0), M(1), M(2),
            m(0), M(1), M(2);

    // Edges of the bounding box
    Eigen::MatrixXi E_box(12,2);
    E_box <<
          0, 1,
            1, 2,
            2, 3,
            3, 0,
            4, 5,
            5, 6,
            6, 7,
            7, 4,
            0, 4,
            1, 5,
            2, 6,
            7 ,3;

    // Plot the mesh
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);

    igl::read_triangle_mesh("data/cube.off",V,F);


    // Plot the corners of the bounding box as points
    viewer.data().add_points(V_box,Eigen::RowVector3d(1,0,0));

    // Plot the edges of the bounding box
    for (unsigned i=0;i<E_box.rows(); ++i)
        viewer.data().add_edges
                (
                        V_box.row(E_box(i,0)),
                        V_box.row(E_box(i,1)),
                        Eigen::RowVector3d(1,0,0)
                );



    // Launch the viewer
    viewer.launch();
}

bool BasicScene::findSmallestBox(igl::AABB<Eigen::MatrixXd ,3> tree1, igl::AABB<Eigen::MatrixXd ,3> tree2){
    if(isCollide(tree1,tree2)){
            if(!isCollide(*tree1.m_right,*tree2.m_right) &&
               !isCollide(*tree1.m_right, *tree2.m_left) &&
               !isCollide(*tree1.m_left,*tree2.m_left) &&
               !isCollide(*tree1.m_left,*tree2.m_right)){
                return true;
            }
            else if(tree1.is_leaf() && !tree2.is_leaf())
                return findSmallestBox(tree1,*tree2.m_left) || findSmallestBox(tree1,*tree2.m_right);
            else if(!tree1.is_leaf() && tree2.is_leaf())
                return findSmallestBox(*tree1.m_left,tree2) || findSmallestBox(*tree1.m_right,tree2);
            else
                return findSmallestBox(*tree1.m_right,*tree2.m_right) ||
                       findSmallestBox(*tree1.m_right, *tree2.m_left) ||
                       findSmallestBox(*tree1.m_left,*tree2.m_left) ||
                       findSmallestBox(*tree1.m_left,*tree2.m_right);
    }

//    while (!stack.empty()){
//        stack.top();
//    }
//    if(isCollide(tree1,tree2)) {
//        stack.push(tree1);
//        stack.push(tree2);
//        while (!stack.empty()){
//            igl::AABB<Eigen::Matrix<double,-1,-1,0>,3> currtree1=stack.top();
//            igl::AABB<Eigen::Matrix<double,-1,-1,0>,3> currtree2=stack.top();
//
//            if(isCollide(currtree1,currtree2)){
//                if(currtree1.is_leaf() && currtree2.is_leaf()){
//                    stack.push(currtree1);
//                    stack.push(currtree2);
//                    return true;
//                }
//                else if(currtree1.is_leaf() && !currtree2.is_leaf()){
//                    stack.push(currtree1);
//                    stack.push(*currtree2.m_right);
//                    stack.push(currtree1);
//                    stack.push(*currtree2.m_left);
//                }else if(!currtree1.is_leaf() && currtree2.is_leaf()){
//                    stack.push(currtree2);
//                    stack.push(*currtree1.m_right);
//                    stack.push(currtree2);
//                    stack.push(*currtree1.m_left);
//                } else{
//                    stack.push(*tree1.m_left);
//                    stack.push(*tree2.m_right);
//                    stack.push(*tree1.m_left);
//                    stack.push(*tree2.m_left);
//                    stack.push(*tree1.m_right);
//                    stack.push(*tree2.m_right);
//                    stack.push(*tree1.m_right);
//                    stack.push(*tree2.m_left);
//                }
//            }
//        }
//        return false;
//    }






//        if(tree1.is_leaf() && tree2.is_leaf()){
//            return true;
//        }
//        else if(tree1.is_leaf() && !tree2.is_leaf())
//            return findSmallestBox(tree1,*tree2.m_left) || findSmallestBox(tree1,*tree2.m_right);
//        else if(!tree1.is_leaf() && tree2.is_leaf())
//            return findSmallestBox(*tree1.m_left,tree2) || findSmallestBox(*tree1.m_right,tree2);
//        else{
//            return findSmallestBox(*tree1.m_right,*tree2.m_right) ||
//                    findSmallestBox(*tree1.m_right, *tree2.m_left) ||
//                    findSmallestBox(*tree1.m_left,*tree2.m_left) ||
//                    findSmallestBox(*tree1.m_left,*tree2.m_right);
//        }

}

bool BasicScene::isCollide(igl::AABB<Eigen::MatrixXd,3> tree1, igl::AABB<Eigen::MatrixXd,3> tree2){
//    auto mesh1 = bunny1->GetMeshList();
//    auto mesh2= bunny2->GetMeshList();
//    V1 = mesh1[0]->data[0].vertices;
//    F1 = mesh1[0]->data[0].faces;
//    V2 = mesh2[0]->data[0].vertices;
//    F2 = mesh2[0]->data[0].faces;

    Eigen::Matrix<float,4,4> T1= bunny1->GetTransform();
    Eigen::Matrix<float,4,4> T2= bunny2->GetTransform();


    Eigen::Matrix<float,4,1> new1={tree1.m_box.min().x(), tree1.m_box.min().y(), tree1.m_box.min().z(), 1};
    Eigen::Matrix<float,4,1> newmin1=T1*new1;
    Eigen::Matrix<float,4,1> new2={tree1.m_box.max().x(), tree1.m_box.max().y(), tree1.m_box.max().z(), 1};
    Eigen::Matrix<float,4,1> newmax1=T1*new2;
    Eigen::Matrix<float,4,1> new3={tree2.m_box.min().x(), tree2.m_box.min().y(), tree2.m_box.min().z(), 1};
    Eigen::Matrix<float,4,1> newmin2=T2*new1;
    Eigen::Matrix<float,4,1> new4={tree2.m_box.max().x(), tree2.m_box.max().y(), tree2.m_box.max().z(), 1};
    Eigen::Matrix<float,4,1> newmax2=T2*new2;
    double minx1=newmin1.x();
    double miny1=newmin1.y();
    double maxx1=newmax1.x();
    double maxy1=newmax1.y();

    double minx2=newmin2.x();
    double miny2=newmin2.y();
    double maxx2=newmax2.x();
    double maxy2=newmax2.y();

    if(maxx1 > minx2 &&
       minx1 < maxx2 &&
       maxy1 > miny2 &&
       miny1 < maxy2){
        return true;
    }
    return false;

}

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create( "camera", fov, float(width) / height, near, far);

    AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    auto daylight{std::make_shared<Material>("daylight", "shaders/cubemapShader")};
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{Model::Create("background", Mesh::Cube(), daylight)};
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();


    auto program = std::make_shared<Program>("shaders/basicShader");
    auto material{ std::make_shared<Material>("material", program)}; // empty material
//    SetNamedObject(cube, Model::Create, Mesh::Cube(), material, shared_from_this());

 //   material->AddTexture(0, "textures/box0.bmp", 2);
    auto bunnyMesh{IglLoader::MeshFromFiles("bunny_igl", "data/bunny.off")};


    bunny1 = Model::Create( "bunny1",bunnyMesh, material);
    bunny2 = Model::Create( "bunny2", bunnyMesh, material);
    AddChild(bunny1);
    AddChild(bunny2);
    bunny1->Scale(10);
    bunny1->Translate({-2,0,0});
    bunny2->Translate({2,0,0});
    bunny2->Scale(10);
    bunny1->showWireframe=true;
    bunny2->showWireframe=true;
    bunny1->showTextures=true;
    bunny1->isPickable=true;
    bunny2->isPickable=true;

    camera->Translate(15, Axis::Z);



    bunny_velocity={1,0,0};
    auto mesh1 = bunny1->GetMeshList();
    auto mesh2= bunny2->GetMeshList();
//    Eigen::VectorXi EMAP;
//    Eigen::MatrixXi F,E,EF,EI;
//    Eigen::VectorXi EQ;
//  // If an edge were collapsed, we'd collapse it to these points:
//    Eigen::MatrixXd V, C;
//    int num_collapsed;
//
//  // Function to reset original mesh and data structures
    V1 = mesh1[0]->data[0].vertices;
    F1 = mesh1[0]->data[0].faces;
    V2 = mesh2[0]->data[0].vertices;
    F2 = mesh2[0]->data[0].faces;

    tree1.init(V1,F1);
    tree2.init(V2,F2);


    //Rendering the bounding box of the bunnies
    cube1 = Model::Create("cube", Mesh::Cube(), material);
    cube2 = Model::Create("cube", Mesh::Cube(), material);

    Eigen::Matrix<float,4,4> T1= bunny1->GetTransform();
    Eigen::Matrix<float,4,4> T2= bunny2->GetTransform();

    auto meshcube = cube1->GetMeshList();

//////////////////////////////
    //RenderBoundingBox(V1,F1);
/////////////////////////////
    AddChild(cube1);
    AddChild(cube2);


    Eigen::Matrix<float,3,1> center={tree1.m_box.center().x(),tree1.m_box.center().y(),tree1.m_box.center().z()};
    Eigen::Matrix<float,4,1> newcenter=T1*newcenter;
//
    cube1->SetCenter(center);
    cube1->SetTransform(T1);
   // cube1->Scale(0.8);
  //  cube1->Translate({-2.3,1,0});
    cube1->showFaces= false;
    cube2->Translate({1.7,1,0});
    cube2->Scale(0.8);
    cube2->showFaces= false;

    cubeV=Mesh::Cube()->data[0].vertices;


    //igl::read_triangle_mesh("data/cube.off",V1,F1);

//    tree2.init(V2,F2);
//    std::list<Eigen::AlignedBox<double,3>::CornerType> P1;
//    P1.push_back(tree2.m_box.BottomLeftCeil);
//    P1.push_back(tree2.m_box.BottomLeftFloor);


//   // igl::read_triangle_mesh("data/cube.off",V,F);
//    igl::edge_flaps(F,E,EMAP,EF,EI);
//    std::cout<< "vertices: \n" << V <<std::endl;
//    std::cout<< "faces: \n" << F <<std::endl;
//
//    std::cout<< "edges: \n" << E.transpose() <<std::endl;
//    std::cout<< "edges to faces: \n" << EF.transpose() <<std::endl;
//    std::cout<< "faces to edges: \n "<< EMAP.transpose()<<std::endl;
//    std::cout<< "edges indices: \n" << EI.transpose() <<std::endl;

}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 1.0f, 1.0f, 1.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 1.0f, 1.0f, 1.0f);
    //cube->Rotate(0.01f, Axis::All);

    Eigen::Matrix<float,4,4> T1= bunny1->GetTransform();
    Eigen::Matrix<float,4,4> T2= bunny2->GetTransform();
    cube1->SetTransform(T1);
    cube2->SetTransform(T2);





        bunny1->Translate(0.05*(bunny_velocity.at(0)),Axis::X);
        bunny1->Translate(0.05*(bunny_velocity.at(1)),Axis::Y);
//        cube1->Translate(0.05*(bunny_velocity.at(0)),Axis::X);
//        cube1->Translate(0.05*(bunny_velocity.at(1)),Axis::Y);



    if(findSmallestBox(tree1,tree2)){
        bunny_velocity={0,0,0};
    }


}

void BasicScene::KeyCallback(cg3d::Viewport *_viewport, int x, int y, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {

        if (key == GLFW_KEY_UP) {
            bunny_velocity={0,1,0};
        } else if (key == GLFW_KEY_DOWN) {
            bunny_velocity={0,-1,0};
        } else if(key==GLFW_KEY_RIGHT){
            bunny_velocity={1,0,0};
        } else if(key==GLFW_KEY_LEFT){
            bunny_velocity={-1,0,0};
        } else if(key==GLFW_KEY_1){
            pickedModel=bunny1;
        }

    }
}

