#ifndef ASYNC_BUFFER_H
#define ASYNC_BUFFER_H

/*
This is a class that implement a single-producer-single-consumer (SPSC) queue.
The implementation of this queue is based on a circular buffer using fixed-length array.

*/

//
#include <iostream>
#include <vector>
#include <utility> // std::pair, std::make_pair
#include <mutex>

using std::vector;

template <class _T>
class async_buffer{
public:
    async_buffer(size_t buffer_length_in);
    async_buffer(size_t buffer_length_in, _T place_holder_element);

    // Public method for operating the queue
    bool put(const _T & element_in);
    std::pair<_T, bool> front();
    bool pop();


    // Status of the queue
    // The following method has suttle mutex setting that may effect the result
    bool is_empty(); // This is a fast method which may return true even when there are some elements in queue (but not vise versa)
    bool is_full();  // // This is a fast method which may return true even when there are some sapces in queue (but not vise versa)
    int size_est(); // The number of buffered contents. This is a fast method with no mutex but may be incorrect in both direction.
    size_t size_exact(); // The number of buffered contents. This is a blocking method with mutex, which will return a correct value.


private:
    // Parameters
    size_t _dl_len;

    // The container
    vector<_T> _data_list;

    // The indicators
    int _idx_write;
    int _idx_read;

    // mutex locks
    std::mutex * _mlock_idx_write;
    std::mutex * _mlock_idx_read;
    // std::mutex _mlock_cal_size;

    // Private methods
    inline void _set_index_write(int idx_write_new){
        {
            std::lock_guard<std::mutex> _lock(_mlock_idx_write);
            _idx_write = idx_write_new;
        }
    }
    inline void _set_index_read(int idx_read_new){
        {
            std::lock_guard<std::mutex> _lock(_mlock_idx_read);
            _idx_read = idx_read_new;
        }
    }


    // utilities
    inline int _increase_idx(int idx_in){
        // Calculate the increased index, not setting the index
        return _correcting_idx(idx_in+1);
    }
    inline int _correcting_idx(int idx_in){
        // The following equation is to correct the behavior of negative value
        // -7 % 3 = -1 --> -7 mod 3 = 2
        return ( ( _dl_len + (idx_in % _dl_len) ) % _dl_len );
    }
    inline int _cal_size(int _idx_write_in, int _idx_read_in){
        // Calculate the number of buffered elements according to the indexes given.
        return _correcting_idx(_idx_write_in - _idx_read_in);
    }
    inline bool _is_empty(int _idx_write_in, int _idx_read_in){
        // Determine if the given indexes indicate that the buffer is full
        return ( _cal_size(_idx_write_in, _idx_read_in) == 0 );
    }
    inline bool _is_full(int _idx_write_in, int _idx_read_in){
        // Determine if the given indexes indicate that the buffer is full
        return ( _cal_size(_idx_write_in, _idx_read_in) == (_dl_len-1) );
    }

};


#endif
