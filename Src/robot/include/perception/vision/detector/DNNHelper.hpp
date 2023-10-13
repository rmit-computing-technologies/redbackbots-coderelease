
#pragma once

#include <string>
#include <iostream>
#include <vector>

#include <tiny_dnn/util/util.h>

using namespace tiny_dnn;

void Read_File(float *Indata, int size,const char *FileName);
void Read_File(float *Indata, int size, std::string  FileName);
void Read_File(tiny_dnn::vec_t &Indata, const std::string);

