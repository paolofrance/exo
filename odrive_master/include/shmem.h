#pragma once

#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <iostream>
#include <semaphore.h>

#define IPC_ERROR_CODE -1

/**
 * \class shared_memory
 * \brief Header-only C++ implementation for Linux of shared memory based on POSIX
 * 
 */
class shared_memory {

public:
    /// @brief  Empty constructor with filename (const std::string reference)
    /// @param fname 
    shared_memory(const std::string& fname = "/elio") : fname(fname) 
    {
    }

    /// @brief Empty constructor with block size (bytes)
    /// @param size memory block size in bytes
    shared_memory(size_t size) : BLOCK_SIZE(size)
    {
    }

    /**
     * @brief Construct a new shared memory object and attach the memory block
     * 
     * @param fname filename for the memory block
     * @param size size of the memory block (bytes)
     */
    shared_memory(const std::string& fname, size_t size, bool create = true) : fname(fname), BLOCK_SIZE(size)
    {
        attach_memory_block(create);
    }

    /**
     * @brief Destroy the shared memory object and detach the memory block
     * 
     */
    ~shared_memory() 
    {
        detach_memory_block();
    }

    /**
     * @brief Init the shared memory by setting filename and block size. Use this with the empty constructor
     * 
     * @param name filename for the memory block
     * @param size size of the memory block (bytes)
     */
    void init(const std::string& name, size_t size) {

        fname = name;
        BLOCK_SIZE = size;

    }

    /// @brief  Get the block_id from ftok
    int get_block_id() const {

        return block_id;
    
    }

    /**
     * @brief Get the pointer to the shared memory buffer
     * 
     * @return uint8_t* pointer to the shared memory buffer (unsigned byte)
     */
    uint8_t* get() const {

        if( attached ) return p_mem;
        else return NULL;

    }

    /**
     * @brief Attach or create the memory block for the shared memory array
     * 
     * @param create whether to flag IPC_CREAT (open or create) or not (open only)
     * @return uint8_t* 
     */
    uint8_t* attach_memory_block(bool create) {

        if( attached ) return p_mem;
        if( !allocated ) get_shared_block(create);

        if( block_id == IPC_ERROR_CODE ) return NULL;

        // map the shared block of memory into the calling's process memory
        // and assign it to the pointer "p_mem"
        p_mem = (uint8_t*) shmat(block_id, NULL, 0);
        if( p_mem == (uint8_t*) IPC_ERROR_CODE ) return nullptr;
        attached = true;

        return p_mem;

    }

    /**
     * @brief Put the variable pointed by var into the shared memory array
     * 
     * @param var uint8_t* pointer (unsigned byte) to the object to put into the shmem
     * @param count_bytes number of bytes of the object pointed by var
     */
    void put(uint8_t* var, size_t count_bytes) {

        if( p_mem == NULL ) return;
        
        // put the variable pointed by var into the shared memory buffer
        memcpy(p_mem+counter, var, count_bytes);
        // update the number of stored bytes
        counter += count_bytes;

    }

    /**
     * @brief Write the variable pointed by var into the shared memory array
     * 
     * @param var uint8_t* pointer (unsigned byte) to the object to write into the shmem
     * @param count_bytes number of bytes of the object pointed by var
     * @param offset offset position into the array to read from (default = 0)
     */
    void write(uint8_t* var, size_t count_bytes, size_t offset = 0) {

        if( p_mem == NULL ) return;

        // put the variable pointed by var into the shared memory buffer at the specified position
        // this may overwrite previous content
        memcpy(p_mem+offset, var, count_bytes);
        // update the number of stored bytes; call shared_memory::clear() to remove these bytes
        counter += count_bytes;
        
    }

    /**
     * @brief Clear the memory previously written with shared_memory::write(). 
     * WARNING: do not mix write()/clear() and put()/remove() calls.
     * 
     * @param count_bytes number of bytes to clear (set to 0)
     * @param offset position offset in the shared memory array
     */
    void clear(size_t count_bytes, size_t offset = 0) {

        if( p_mem == NULL ) return;

        // clear the number of specified bytes (i.e., set to 0)
        memset(p_mem+offset, 0, count_bytes);
        // update the number of stored bytes
        counter -= count_bytes;

    }

    /**
     * @brief Non-destructive read from the shared memory array (i.e., does not clear contents)
     * 
     * @param var uint8_t* pointer (unsigned byte) to the object to put into the shmem
     * @param count_bytes number of bytes of the object pointed by var
     * @param offset offset position into the array to read from (default = 0)
     */
    void read(uint8_t* var, size_t count_bytes, size_t offset = 0) const {

        // read the variable at p_mem[offset] and put it into *var
        memcpy(var, (uint8_t*) (((uint8_t*) p_mem)+offset), count_bytes);

    }

    /**
     * @brief Destructive read from the shared memory array (i.e., read and clear)
     * 
     * @param var uint8_t* pointer (unsigned byte) to the object to put into the shmem
     * @param count_bytes number of bytes of the object pointed by var
     * @param offset offset position into the array to read from (default = 0)
     */
    void remove(uint8_t* var, size_t count_bytes, size_t offset = 0) {

        // read and delete from the shared memory
        read(var, count_bytes, offset);
        // clear the bytes
        memset((uint8_t*) (((uint8_t*) p_mem)+offset), 0, count_bytes);
        // update the number of stored bytes
        counter -= count_bytes;

    }

    /**
     * @brief Get the number of bytes stored in the shared memory buffer
     * 
     * @return size_t 
     */
    size_t get_size() const {

        return counter;

    }

    /// @brief  Detach the shared memory object and return true if success
    bool detach_memory_block() {

        // this does not affect the shared memory block at all
        // it just detaches it from the calling process

        int r = shmdt(p_mem);
        if( r != IPC_ERROR_CODE ) {

            p_mem = NULL;
            attached = false;

        }
        return ( shmdt(p_mem) != IPC_ERROR_CODE);

    }

    /// @brief  Check whether this is attached to the shared memory object
    bool check_attached() const {
        return attached;
    }

    /// @brief  Destroy the shared memory object (i.e., un-allocates the memory) and returns true if successful
    bool destroy() {

        if( block_id == IPC_ERROR_CODE) return false;

        int r = shmctl(block_id, IPC_RMID, NULL);

        if( r != IPC_ERROR_CODE ) {
            attached = false;
            allocated = false;
            return true;
        }
        else{
            return false;
        }

    }

private:
    int get_shared_block(bool create) {

        key_t key = ftok(fname.c_str(), 0);
        block_id = IPC_ERROR_CODE;
        if( key == IPC_ERROR_CODE ) return IPC_ERROR_CODE;

        allocated = true;

        int flag = 0644;
        if( create ) flag |= IPC_CREAT;
        block_id = shmget(key, BLOCK_SIZE, flag);

    }

public:
    /// @brief  Filename for 'ftok' to create the shared memory object 
    std::string fname;
    /// @brief  Memory size in bytes
    size_t BLOCK_SIZE = 4096;
    /// @brief  Number of stored bytes; private, to be accessed with: shared_memory::get_size()
    size_t counter = 0;

private:
    bool allocated = false;
    bool attached = false;
    int block_id = 0;
    
    /// @brief  Pointer to the shared memory array; private, to be accessed with shared_memory::get()
    uint8_t* p_mem;
    

};