class_<Odometry>("Odometry")
   .def(init<float, float, float>())
   .def(init<>())
   .add_property("forward", &Odometry::forward)
   .add_property("left", &Odometry::left)
   .add_property("turn", &Odometry::turn)
   .def("clear", &Odometry::clear)
   .def(self + self)
   .def(self - self);
