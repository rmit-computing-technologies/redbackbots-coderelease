#pragma once

template <typename T, size_t n>
void
array_to_python<T, n>::array_wrap_dealloc(
      array_to_python<T, n>::array_wrap *self)
{
   /* No members to free. */
   PyTypeObject *tp = Py_TYPE(self);
   tp->tp_free(self);
   //below commented out to avoid a segfault, this may need to be added back in 
   //Py_DECREF(tp);
}

template <typename T, size_t n>
PyObject *
array_to_python<T, n>::array_wrap_new(
      PyTypeObject *type,
      PyObject     *args,
      PyObject     *kwds)
{
   /* There are no members visable to python. */
   array_to_python *self;
   self = (array_to_python *)type->tp_alloc(type, 0);
   return (PyObject *)self;
}

template <typename T, size_t n>
int
array_to_python<T, n>::array_wrap_init(
      array_to_python<T, n>::array_wrap *self,
      PyObject                          *args,
      PyObject                          *kwds)
{
   /* Again nothing to do here. */
   return 0;
}

template <typename T, size_t n>
PyObject *
array_to_python<T, n>::array_wrap___getitem__(
      array_to_python<T, n>::array_wrap *self,
      PyObject                          *item)
{
   /* Check for type error */
   if (! PyLong_Check(item)) {
      PyObject *py_class = PyObject_GetAttrString(item, "__class__");
      PyObject *py_name  = PyObject_GetAttrString(py_class, "__name__");


      std::string errstr = "array indicies must be integers, not ";
      //errstr += PyString_AsString(py_name);
      errstr += std::string(PyUnicode_AsUTF8(py_name));

      PyErr_SetString(PyExc_TypeError, errstr.c_str());

      return NULL;
   }

   long i = PyLong_AsLong(item);

   /* Check i is in bounds */
   if (! (0 <= (unsigned long)i && (unsigned long)i < n)) {
      PyErr_SetString(PyExc_ValueError, "array index out of range");
      return NULL;
   }

   /* Use boost::python to create the correct python object type
    * and then steal the reference.
    */
   boost::python::object ret(self->p[i]);
   boost::python::incref(ret.ptr());

   return ret.ptr();
}

template <typename T, size_t n>
Py_ssize_t
array_to_python<T, n>::array_wrap___len__(
      array_to_python<T, n>::array_wrap *self)
{
   return n;
}

template <typename T, size_t n>
PyMappingMethods array_to_python<T, n>::array_wrap_mapping = {
   (lenfunc   )array_wrap___len__,
   (binaryfunc)array_wrap___getitem__,
   NULL  /* Not Settable */
};

// http://docs.python.org/3/c-api/typeobj.html
template <typename T, size_t n>
PyTypeObject array_to_python<T, n>::array_wrapType = {
   PyVarObject_HEAD_INIT(NULL, 0)
   .tp_name = "robot.array_wrap",
   .tp_basicsize = sizeof(array_wrap),
   .tp_doc = "Wrapper for C arrays",
   .tp_new = array_wrap_new,
   .tp_dealloc = (destructor)array_wrap_dealloc,
   .tp_as_mapping = &array_wrap_mapping,
   .tp_flags = Py_TPFLAGS_DEFAULT,
   .tp_init = (initproc)array_wrap_init,
};

//template <typename T, size_t n>
//PyTypeObject array_to_python<T, n>::array_wrapType = {
//   PyObject_HEAD_INIT(NULL)
//   0,                              /* ob_size*/
//   "robot.array_wrap",             /* tp_name*/
//   sizeof(array_wrap),             /* tp_basicsize*/
//   0,                              /* tp_itemsize*/
//   (destructor)array_wrap_dealloc, /* tp_dealloc*/
//   0,                              /* tp_print*/
//   0,                              /* tp_getattr*/
//   0,                              /* tp_setattr*/
//   0,                              /* tp_compare*/
//   0,                              /* tp_repr*/
//   0,                              /* tp_as_number*/
//   0,                              /* tp_as_sequence*/
//   &array_wrap_mapping,            /* tp_as_mapping*/
//   0,                              /* tp_hash */
//   0,                              /* tp_call*/
//   0,                              /* tp_str*/
//   0,                              /* tp_getattro*/
//   0,                              /* tp_setattro*/
//   0,                              /* tp_as_buffer*/
//   Py_TPFLAGS_DEFAULT,             /* tp_flags*/
//   "Wrapper for C arrays",         /* tp_doc */
//   0,                              /* tp_traverse */
//   0,                              /* tp_clear */
//   0,                              /* tp_richcompare */
//   0,                              /* tp_weaklistoffset */
//   0,                              /* tp_iter */
//   0,                              /* tp_iternext */
//   0,                              /* tp_methods */
//   0,                              /* tp_members */
//   0,                              /* tp_getset */
//   0,                              /* tp_base */
//   0,                              /* tp_dict */
//   0,                              /* tp_descr_get */
//   0,                              /* tp_descr_set */
//   0,                              /* tp_dictoffset */
//   (initproc)array_wrap_init,      /* tp_init */
//   0,                              /* tp_alloc */
//   array_wrap_new,                 /* tp_new */
//};
