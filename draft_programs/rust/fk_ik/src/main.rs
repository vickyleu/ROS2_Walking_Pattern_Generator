use ndarray::*;

fn main() {
  println!("Hello, world!");

  let mut theta_rwl: f64 = 0.0;  // Right Waist Roll
  let mut theta_rwp: f64 = 0.0;  // Right Waist Pitch
  let mut theta_rkp: f64 = 0.0;  // Right Knee Pitch
  let mut _theta_lwl: f64 = 0.0;  // Left Waist Roll
  let mut _theta_lwp: f64 = 0.0;  // Left Waist Pitch
  let mut _theta_lkp: f64 = 0.0;  // Left Knee Pitch

  let vp_c_rwl: Array1<f64> = array![-0.091, -0.075, 0.];
  let vp_rwl_rwp: Array1<f64> = array![0.091, -0.055, 0.];
  let vp_rwp_rkp: Array1<f64> = array![0., 0., -0.16];
  let vp_rkp_re: Array1<f64> = array![0., 0., -0.173];

  let vp_c_re: Array1<f64> = array![0.0, -0.075 + -0.055, -(0.16_f64.powf(2.) + 0.173_f64.powf(2.)).sqrt()];  // target position
  println!("vp_c_re: {}", &vp_c_re);

// IK
  //let el1: i32 = ((&vp_c_re[1] - &vp_c_rwl[1]) * 1000.0) as i32;
  //let el2: f64 = el1 as f64 / 1000.;  // 丸め誤差だかが原因でacos()がnanになるので、i32に変換してからfloatに戻す
  theta_rwl = std::f64::consts::PI / 2. - &vp_c_re[2].abs().atan2(&vp_c_re[1] - (&vp_c_rwl[1] + &vp_rwl_rwp[1]));
  println!("theta_rwl: {}", &theta_rwl * 180. / std::f64::consts::PI);

  // let rm_rwl: Array2<f64> = array![
  //   [1., 0., 0.,],
  //   [0., theta_rwl.cos(), -theta_rwl.sin()],
  //   [0., theta_rwl.sin(), theta_rwl.cos()]
  // ];
  let T_wr: Array2<f64> = array![
    [1., 0.             , 0.              , -0.091],
    [0., theta_rwl.cos(), -theta_rwl.sin(), -0.075],
    [0., theta_rwl.sin(),  theta_rwl.cos(),  0.],
    [0., 0.             , 0.              ,  1.]
  ];
  let vv: Array2<f64> = array![[vp_rwl_rwp[0]], [vp_rwl_rwp[1]], [vp_rwl_rwp[2]], [1.]];
  let vp_c_rwp: Array2<f64> = T_wr.dot(&vv);
  println!("vp_c_rwp: {}", &vp_c_rwp.t());
  let vr: Array1<f64> = &vp_c_re - array![vp_c_rwp[[0,0]], vp_c_rwp[[1,0]], vp_c_rwp[[2,0]]];
  println!("vr: {}", &vr);
  let a: f64 = vp_rwp_rkp[2].abs();
  let b: f64 = vp_rkp_re[2].abs();
  let c: f64 = (&vr[0].powf(2.) + &vr[1].powf(2.) + &vr[2].powf(2.)).sqrt();
  println!("c: {}",&c);
  theta_rkp = -(((&a.powf(2.) + &b.powf(2.) - &c.powf(2.))/(2.*&a*&b)).acos()) + std::f64::consts::PI;
  let foo: f64 = vr[0].atan2((&vr[1].powf(2.) + &vr[2].powf(2.)).sqrt());  // 目標X軸座標と重心X座標を結ぶ直線の傾きがZ軸となす角度
  let bar: f64 = ((&b*(std::f64::consts::PI - &theta_rkp).sin())/(&c)).asin();  // 腰Yaw軸から足先までからなる三角形での腰Yaw軸にあたる内角の角度
  println!("foo: {}", &foo * 180. / std::f64::consts::PI);
  println!("bar: {}", &bar * 180. / std::f64::consts::PI);
  theta_rwp = -1. * (foo + bar);  // 回転方向の修正
  println!("theta_rwp: {}", &theta_rwp * 180. / std::f64::consts::PI);
  println!("theta_rkp: {}", &theta_rkp * 180. / std::f64::consts::PI);

// FK
  // let T_wr: Array2<f64> = array![
  //   [1., 0.             , 0.              , -0.091],
  //   [0., theta_rwl.cos(), -theta_rwl.sin(), -0.075],
  //   [0., theta_rwl.sin(),  theta_rwl.cos(),  0.],
  //   [0., 0.             , 0.              ,  1.]
  // ];
  let T_wp: Array2<f64> = array![
    [ theta_rwp.cos(), 0., theta_rwp.sin(),  0.091],
    [ 0.             , 1., 0.             , -0.055],
    [-theta_rwp.sin(), 0., theta_rwp.cos(),  0.],
    [ 0.             , 0., 0.             ,  1.]
  ];
  let T_kp: Array2<f64> = array![
    [ theta_rkp.cos(), 0., theta_rkp.sin(),  0.],
    [ 0.             , 1., 0.             ,  0.],
    [-theta_rkp.sin(), 0., theta_rkp.cos(), -0.16],
    [ 0.             , 0., 0.             ,  1.]
  ];
  let T_e: Array2<f64> = array![[0.], [0.], [-0.173], [1.]];

  let fk_result: Array2<f64> = T_wr.dot(&T_wp.dot(&T_kp.dot(&T_e)));
  println!("fk_result: {}", fk_result.t());
}
