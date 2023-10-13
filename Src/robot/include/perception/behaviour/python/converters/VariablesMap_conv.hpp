#pragma once

#include <vector>

#include <boost/python.hpp>
#include <boost/python/refcount.hpp>
#include <boost/program_options/variables_map.hpp>

/**
 * @deprecated vector_indexing_suite is better
 */
template<typename T>
struct Vector_T_to_python {
    static PyObject *convert(const std::vector<T> &v) {
        boost::python::list l;
        for (const auto &e: v) {
            l.append(e);
        }
        return boost::python::incref(l.ptr());
    }
};

struct VariablesMap_to_python {
    static PyObject *convert(const boost::program_options::variables_map &v) {
        boost::python::dict d;
        for (const auto &e: v) {
            if (e.second.value().type() == typeid(std::string)) {
                d[e.first] = e.second.as<std::string>();
            } else if (e.second.value().type() == typeid(int)) {
                d[e.first] = e.second.as<int>();
            } else if (e.second.value().type() == typeid(float)) {
                d[e.first] = e.second.as<float>();
            } else if (e.second.value().type() == typeid(double)) {
                d[e.first] = e.second.as<double>();
            } else if (e.second.value().type() == typeid(bool)) {
                d[e.first] = e.second.as<bool>();
            }
        }
        return boost::python::incref(d.ptr());
    }
};