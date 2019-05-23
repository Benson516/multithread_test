#include "async_buffer.hpp"


template <class _T>
async_buffer<_T>::async_buffer(size_t buffer_length_in){

    _mlock_idx_write = new std::mutex();
    _mlock_idx_read = new std::mutex();
    // The size is equals to mod(_idx_write - _idx_read, _dl_len)
    // If two indexes are identical, it means that the buffer is empty.
    // Note: we should prevent the case that the buffer is overwitten to "empty" when the buffer is full.
    _idx_read = 0;
    _idx_write = 0;

    // Note: buffer_length_in should be at least "1", or the queue will not store any thing.
    // However, specifying "0" will not cause any error, thus remove the following correcting term.
    /*
    if (buffer_length_in < 1)
        buffer_length_in = 1;
    */
    _dl_len = buffer_length_in + 1; // The _data_list should always contains an place wich is ready to be written, hence it will be larger than the buffer length.
    _data_list.resize(_dl_len);

}
template <class _T>
async_buffer<_T>::async_buffer(size_t buffer_length_in, _T place_holder_element){
    //------------------------------------------------//
    // A place_holder_element is supported at input
    // incase that the element does not has empty constructor.
    //------------------------------------------------//

    // The size is equals to mod(_idx_write - _idx_read, _dl_len)
    // If two indexes are identical, it means that the buffer is empty.
    // Note: we should prevent the case that the buffer is overwitten to "empty" when the buffer is full.
    _idx_read = 0;
    _idx_write = 0;

    // Note: buffer_length_in should be at least "1", or the queue will not store any thing.
    // However, specifying "0" will not cause any error
    _dl_len = buffer_length_in + 1; // The _data_list should always contains an place wich is ready to be written, hence it will be larger than the buffer length.
    _data_list.resize(_dl_len, place_holder_element);

}

//
template <class _T> bool async_buffer<_T>::put(const _T & element_in){
    // To put an element into the buffer
    if (is_full()){
        return false;
    }
    // else
    int _idx_write_tmp;
    {
        std::lock_guard<std::mutex> _lock(&_mlock_idx_write);
        _idx_write_tmp = _idx_write;
    }

    // Note: the copy method may not sussess if _T is "Mat" from opencv
    //       be sure to use IMG.clone() mwthod for putting an image in.
    // The following operation might be time consumming
    _data_list[_idx_write_tmp] = element_in;

    // Note: the following function already got a lock,
    // don't use the same lock recursively
    _set_index_write( _increase_idx(_idx_write_tmp) );
    return true;
}

template <class _T> std::pair<_T, bool> async_buffer<_T>::front(){
    // To get an element from the buffer

    // If the buffer is empty, we
    if (is_empty()){
        return std::pair<_T, bool>(_data_list[0], false);
    }
    // else
    int _idx_read_tmp;
    {
        std::lock_guard<std::mutex> _lock(&_mlock_idx_read);
        _idx_read_tmp = _idx_read;
    }

    // Note: the copy method may not sussess if _T is "Mat" from opencv
    //       be sure to use IMG.clone() mwthod outside this function.
    // The following operation might be time consumming
    return std::pair<_T, bool>(_data_list[_idx_read_tmp], true);
}

template <class _T> bool async_buffer<_T>::pop(){
    // To remove an element from the buffer
    if (is_empty()){
        return false;
    }
    // else
    int _idx_read_tmp;
    {
        std::lock_guard<std::mutex> _lock(&_mlock_idx_read);
        _idx_read_tmp = _idx_read;
    }

    // Note: the following function already got a lock,
    // don't use the same lock recursively
    _set_index_read( _increase_idx(_idx_read_tmp) );
    return true;
}





//
template <class _T> bool async_buffer<_T>::is_empty(){
    // Note: This method is used by "consumer"
    // This is a fast method which may return true even when there are some elements in queue (but not vise versa)

    // Cache the "write" first
    int _idx_write_tmp, _idx_read_tmp;
    {
        std::lock_guard<std::mutex> _lock_w(&_mlock_idx_write);
        _idx_write_tmp = _idx_write;
    }
    {
        std::lock_guard<std::mutex> _lock_r(&_mlock_idx_read);
        _idx_read_tmp = _idx_read;
    }

    return _is_empty(_idx_write_tmp, _idx_read_tmp);
}

template <class _T> bool async_buffer<_T>::is_full(){
    // Note: This method is used by "producer"
    // // This is a fast method which may return true even when there are some sapces in queue (but not vise versa)

    // Cache the "read" first
    int _idx_write_tmp, _idx_read_tmp;
    {
        std::lock_guard<std::mutex> _lock_r(&_mlock_idx_read);
        _idx_read_tmp = _idx_read;
    }
    {
        std::lock_guard<std::mutex> _lock_w(&_mlock_idx_write);
        _idx_write_tmp = _idx_write;
    }


    return _is_full(_idx_write_tmp, _idx_read_tmp);
}

template <class _T> int async_buffer<_T>::size_est(){
    // This method may be used by both producer and consumer
    // The number of buffered contents. This is a fast method with no mutex but may be incorrect in both direction.

    // Cache the "read" first, since the write might change more frequently
    int _idx_write_tmp, _idx_read_tmp;
    {
        std::lock_guard<std::mutex> _lock_r(&_mlock_idx_read);
        _idx_read_tmp = _idx_read;
    }
    {
        std::lock_guard<std::mutex> _lock_w(&_mlock_idx_write);
        _idx_write_tmp = _idx_write;
    }
    return _cal_size(_idx_write_tmp, _idx_read_tmp);
}

template <class _T> size_t async_buffer<_T>::size_exact(){
    // This method may be used by both producer and consumer
    // The number of buffered contents. This is a blocking method with mutex, which will return a correct value.

    // Cache both index at the same time and lock all the way to the end
    std::lock_guard<std::mutex> _lock_w(&_mlock_idx_write);
    std::lock_guard<std::mutex> _lock_r(&_mlock_idx_read);
    int _idx_read_tmp = _idx_read;
    int _idx_write_tmp = _idx_write;

    return size_t( _cal_size(_idx_write_tmp, _idx_read_tmp) );
}
