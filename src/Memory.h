#pragma once
#include <malloc.h>
#include <vector>

// WINDOWS ONLY
#define L1_CACHE_LINE_SIZE 64

template <typename T>
T* AllocateAllined(int count) {
    return (T*)_aligned_malloc(count * sizeof(T), L1_CACHE_LINE_SIZE);
}

void Release(void* ptr) {
  if(!ptr) return;

  _aligned_free(ptr);
}