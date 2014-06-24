#include <igl/readOBJ.h>
#include <igl/readDMAT.h>
#include <igl/viewer/Viewer.h>
#include <igl/barycenter.h>
#include <igl/avg_edge_length.h>
#include <igl/comiso/nrosy.h>
#include <igl/comiso/miq.h>
#include <igl/comiso/frame_field.h>
#include <igl/frame_field_deformer.h>
#include <igl/jet.h>
#include <igl/frame_to_cross_field.h>

// Input mesh
Eigen::MatrixXd V;
Eigen::MatrixXi F;

// Face barycenters
Eigen::MatrixXd B;

// Scale for visualizing the fields
double global_scale;

// Input frame field constraints
Eigen::VectorXi b;
Eigen::MatrixXd bc1;
Eigen::MatrixXd bc2;

// Interpolated frame field
Eigen::MatrixXd FF1, FF2;

// Deformed mesh
Eigen::MatrixXd V_deformed;
Eigen::MatrixXd B_deformed;

// Frame field on deformed
Eigen::MatrixXd FF1_deformed;
Eigen::MatrixXd FF2_deformed;

// Cross field on deformed
Eigen::MatrixXd X1_deformed;
Eigen::MatrixXd X2_deformed;

// Global parametrization
Eigen::MatrixXd V_uv;
Eigen::MatrixXi F_uv;

// Create a texture that hides the integer translation in the parametrization
void line_texture(Eigen::Matrix<char,Eigen::Dynamic,Eigen::Dynamic> &texture_R,
                  Eigen::Matrix<char,Eigen::Dynamic,Eigen::Dynamic> &texture_G,
                  Eigen::Matrix<char,Eigen::Dynamic,Eigen::Dynamic> &texture_B)
{
  unsigned size = 128;
  unsigned size2 = size/2;
  unsigned lineWidth = 3;
  texture_R.setConstant(size, size, 255);
  for (unsigned i=0; i<size; ++i)
    for (unsigned j=size2-lineWidth; j<=size2+lineWidth; ++j)
      texture_R(i,j) = 0;
  for (unsigned i=size2-lineWidth; i<=size2+lineWidth; ++i)
    for (unsigned j=0; j<size; ++j)
      texture_R(i,j) = 0;
  
  texture_G = texture_R;
  texture_B = texture_R;
}

bool key_down(igl::Viewer& viewer, unsigned char key, int modifier)
{
  using namespace std;
  using namespace Eigen;

  if (key <'1' || key >'6')
    return false;

  viewer.clear_mesh();
  viewer.options.show_lines = false;
  viewer.options.show_texture = false;

  if (key == '1')
  {
    // Frame field constraints
    viewer.set_mesh(V, F);

    MatrixXd F1_t = MatrixXd::Zero(FF1.rows(),FF1.cols());
    MatrixXd F2_t = MatrixXd::Zero(FF2.rows(),FF2.cols());
    // Highlight in red the constrained faces
    MatrixXd C = MatrixXd::Constant(F.rows(),3,1);
    for (unsigned i=0; i<b.size();++i)
    {
      C.row(b(i)) << 1, 0, 0;
      F1_t.row(b(i)) = bc1.row(i);
      F2_t.row(b(i)) = bc2.row(i);
    }

    viewer.set_colors(C);

    MatrixXd C1,C2;
    VectorXd K1 = F1_t.rowwise().norm();
    VectorXd K2 = F2_t.rowwise().norm();
    igl::jet(K1,true,C1);
    igl::jet(K2,true,C2);

    viewer.add_edges (B - global_scale*F1_t, B + global_scale*F1_t ,C1);
    viewer.add_edges (B - global_scale*F2_t, B + global_scale*F2_t ,C2);
  }

  if (key == '2')
  {
    // Frame field
    viewer.set_mesh(V, F);
    MatrixXd C1,C2;
    VectorXd K1 = FF1.rowwise().norm();
    VectorXd K2 = FF2.rowwise().norm();
    igl::jet(K1,true,C1);
    igl::jet(K2,true,C2);

    viewer.add_edges (B - global_scale*FF1, B + global_scale*FF1 ,C1);
    viewer.add_edges (B - global_scale*FF2, B + global_scale*FF2 ,C2);

    // Highlight in red the constrained faces
    MatrixXd C = MatrixXd::Constant(F.rows(),3,1);
    for (unsigned i=0; i<b.size();++i)
      C.row(b(i)) << 1, 0, 0;
    viewer.set_colors(C);

  }

  if (key == '3')
  {
    // Deformed with frame field
    viewer.set_mesh(V_deformed, F);
    viewer.add_edges (B_deformed - global_scale*FF1_deformed, B_deformed + global_scale*FF1_deformed ,Eigen::RowVector3d(1,0,0));
    viewer.add_edges (B_deformed - global_scale*FF2_deformed, B_deformed + global_scale*FF2_deformed ,Eigen::RowVector3d(0,0,1));
    viewer.set_colors(RowVector3d(1,1,1));
  }

  if (key == '4')
  {
    // Deformed with cross field
    viewer.set_mesh(V_deformed, F);
    viewer.add_edges (B_deformed - global_scale*X1_deformed, B_deformed + global_scale*X1_deformed ,Eigen::RowVector3d(0,0,1));
    viewer.add_edges (B_deformed - global_scale*X2_deformed, B_deformed + global_scale*X2_deformed ,Eigen::RowVector3d(0,0,1));
    viewer.set_colors(RowVector3d(1,1,1));
  }

  if (key == '5')
  {
    // Deformed with quad texture
    viewer.set_mesh(V_deformed, F);
    viewer.set_uv(V_uv,F_uv);
    viewer.set_colors(RowVector3d(1,1,1));
    viewer.options.show_texture = true;
  }

  if (key == '6')
  {
    // Deformed with quad texture
    viewer.set_mesh(V, F);
    viewer.set_uv(V_uv,F_uv);
    viewer.set_colors(RowVector3d(1,1,1));
    viewer.options.show_texture = true;
  }

  // Replace the standard texture with an integer shift invariant texture
  Eigen::Matrix<char,Eigen::Dynamic,Eigen::Dynamic> texture_R, texture_G, texture_B;
  line_texture(texture_R, texture_G, texture_B);
  viewer.set_texture(texture_R, texture_B, texture_G);

  return false;
}

int main(int argc, char *argv[])
{
  using namespace Eigen;

  // Load a mesh in OBJ format
  igl::readOBJ("../shared/cube.obj", V, F);

  // Compute face barycenters
  igl::barycenter(V, F, B);

  // Compute scale for visualizing fields
  global_scale =  .2*igl::avg_edge_length(V, F);

  // Load constraints
  MatrixXd temp;
  igl::readDMAT("../shared/cube.dmat",temp);

  b   = temp.block(0,0,temp.rows(),1).cast<int>();
  bc1 = temp.block(0,1,temp.rows(),3);
  bc2 = temp.block(0,4,temp.rows(),3);
  
  // Interpolate the frame field
  igl::frame_field(V, F, b, bc1, bc2, FF1, FF2);

  // Deform the mesh to transform the frame field in a cross field
  igl::frame_field_deformer(V,F,FF1,FF2,V_deformed,FF1_deformed,FF2_deformed);

  // Compute face barycenters deformed mesh
  igl::barycenter(V_deformed, F, B_deformed);

  // Find the closest crossfield to the deformed frame field
  igl::frame_to_cross_field(V,F,FF1_deformed,FF2_deformed,X1_deformed);
  
  // Find a smooth crossfield that interpolates the deformed constraints
  MatrixXd bc_x(b.size(),3);
  for (unsigned i=0; i<b.size();++i)
    bc_x.row(i) = X1_deformed.row(b(i));
  
  VectorXd S;
  igl::nrosy(
             V,
             F,
             b,
             bc_x,
             VectorXi(),
             VectorXd(),
             MatrixXd(),
             4,
             0.5,
             X1_deformed,
             S);

  // The other representative of the cross field is simply rotated by 90 degrees
  MatrixXd B1,B2,B3;
  igl::local_basis(V_deformed,F,B1,B2,B3);
  X2_deformed = igl::rotate_vectors(X1_deformed, VectorXd::Constant(1,M_PI/2), B1, B2);

  // Global seamless parametrization
  igl::miq(V_deformed,
           F,
           X1_deformed,
           X2_deformed,
           V_uv,
           F_uv,
           60.0,
           5.0,
           false,
           0);

  igl::Viewer viewer;
  // Plot the original mesh with a texture parametrization
  key_down(viewer,'6',0);

  // Launch the viewer
  viewer.callback_key_down = &key_down;
  viewer.launch();
}