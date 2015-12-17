/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi
 * email:  alessio.rocchi@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef __INDICES_H__
#define __INDICES_H__

#include <cassert>
#include <list>
#include <sstream>
#include <string>
#include <vector>

namespace OpenSoT
{

class Indices {
public:
    /**
     * @brief RowsChunk a vector<unsigned int> of rows which are contiguous
     */
    typedef std::vector<unsigned int> RowsChunk;
    /**
     * @brief ChunkList a list<vector<int>>, a list of contiguous chunks of rows
     * e.g., {{1,2},{4},{10,11,12}}, is a list of 3 chunks of contiguous row indices,
     * where the first chunk has size 2, the second chunk has size 1, the third chunk has size 3
     */
    typedef std::list< RowsChunk > ChunkList;
private:

    ChunkList _contiguousChunks;
    std::list<unsigned int> _rowsList;
    std::vector<unsigned int> _rowsVector;

    /**
     * @brief getNextAdjacentChunk returns an iterator pointing to the last element of a chunk
     * @param searchBegin iterator pointing to the first element where we start looking for chunks
     * @return the iterator pointing to the last element of a chunk
     */
    std::list<unsigned int>::iterator getNextAdjacentChunk(std::list<unsigned int>::iterator searchBegin);

    void generateChunks();

public:
    /**
     * @brief SubTaskMap creates a SubTaskMap of size 1, with just one index
     * @param i
     */
    Indices(unsigned int i);

    Indices(const std::list<unsigned int>& rowsList);

    Indices(const std::vector<unsigned int> &rowsVector);

    template<class Iterator>
    Indices(Iterator it, const Iterator end) {
        while( it != end)
        {
            _rowsList.push_back(*it);
            ++it;
        }
    }

    Indices(const Indices& indices);

    const ChunkList &getChunks() const;

    /**
     * @brief asList returns the list of all rows as a list (first row has index 0)
     * @return a list of row indices (starting from 0)
     */
    const std::list<unsigned int> &asList() const;

    /**
     * @brief asVector returns the list of all rows as a vector (first row has index 0)
     * @return  a vector of row indices (starting from 0)
     */
    const std::vector<unsigned int> &asVector() const;

    /**
     * @brief shift shifts all indices by the amount specified
     * @param amount by which all indices will be shift
     */
    Indices& shift(unsigned int amount);

    bool isContiguous() const;

    int size() const;

    /**
     * @brief range creates a range of integers, [from,to]
     * @param from the first index
     * @param to the last index
     * @return an Indices object with a sequence of ints [from,to]
     */
    static Indices range(unsigned int from, unsigned int to);

    /**
     * @brief operator + adds a set of indices to the current indices set
     * @param b the indices set to concatenate
     * @return a bigger indices set (sorted, without duplicates)
     */
    Indices operator+(const Indices& b) const;

    /**
     * @brief operator + adds a new index to the current indices set
     * @param r the index to add to the set
     * @return a bigger indices set (sorted, without duplicates)
     */
    Indices operator+(const unsigned int r) const;

    bool operator==(const Indices& b) const;

    operator std::string() const;

    operator std::list<unsigned int>() const;
};

}

#endif
