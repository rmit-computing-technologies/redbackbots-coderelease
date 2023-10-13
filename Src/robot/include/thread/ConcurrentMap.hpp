#pragma once

#include <map>
#include <mutex>
#include <vector>

/* A thread-safe wrapper around the stl map class
 * Uses a boost::mutex to ensure atomicity */
template <class Key, class Data>
class ConcurrentMap {
public:
   /* Used to index into the map
      * @param key the key to look up in the map
      * @return reference to the value corresonding to the key */
   Data & operator[](const Key& key);
   
   /* Find how many values correspond to a key
      * @param key the key to look up in the map
      * @return the number of corresponding values (0 or 1) since it's not a multimap */
   unsigned int count(const Key& key) const;
   
   /* @return a vectory with a copy of all the keys
      * Useful for thread-safe iteration of keys */
   std::vector<Key> keys() const;

private:
   /* Private instance of the stl map, being wrapped */
   std::map<Key, Data> theMap;
   
   /* Mutex to ensure atomicity of operations on the ConcurrentMap */
   mutable std::mutex theMutex;
};

template <class Key, class Data>
Data & ConcurrentMap<Key, Data>::operator[] (const Key &key) {
   std::lock_guard<std::mutex> lock(theMutex);
   return theMap[key];
}

template <class Key, class Data>
unsigned int ConcurrentMap<Key, Data>::count(const Key& key) const {
   std::lock_guard<std::mutex> lock(theMutex);
   return theMap.count(key);
}

template <class Key, class Data>
std::vector<Key> ConcurrentMap<Key, Data>::keys() const {
   std::vector<Key> keyVec;
   std::lock_guard<std::mutex> lock(theMutex);

   typename std::map<Key, Data>::const_iterator it;

   for (it = theMap.begin(); it != theMap.end(); ++it) {
      keyVec.push_back(it->first);
   }
   return keyVec;
}

