#pragma once

namespace iUtils
{
  template <class T, int Size>
  class CHUNK
  {
  public:
    CHUNK()
    {
      objects = new T[Size];
      cur_obj = 0;
      m_pNext = 0;
    }

    ~CHUNK()
    {
      delete [] objects;
    }

    T * Get()
    {
      if ( cur_obj < Size )
        return objects+(cur_obj++);
      else
        return 0;
    }

    void Clear()
    {
      cur_obj = 0;
    }


    CHUNK *m_pNext;

  private:

    T * objects;
    int cur_obj;
  };

  template <class T, int Chunk_Size = 8>
  class CHUNK_HEAP
  {
    typedef CHUNK< T, Chunk_Size > T_CHUNK;
  public:

    CHUNK_HEAP()
    {
      m_pFirst = m_pLast = 0;
    }

    ~CHUNK_HEAP()
    {
      Free();
    }

    void Free()
    {
      T_CHUNK * cur_chunk = m_pFirst;
      while ( cur_chunk )
      {
        T_CHUNK *next_chunk = cur_chunk->m_pNext;
        delete cur_chunk;
        cur_chunk = next_chunk;
      }
      m_pLast = m_pFirst = 0;
    }

    T * Get()
    {
      if ( !m_pLast ) { m_pFirst = m_pLast = new T_CHUNK; }
      T * ret_obj = m_pLast->Get();
      if ( !ret_obj )
      {
        T_CHUNK * next_chunk = m_pLast->m_pNext;
        if ( !next_chunk )
        {
          next_chunk = new T_CHUNK;
          m_pLast->m_pNext = next_chunk;
        }
        m_pLast->Clear();
        m_pLast = next_chunk;
        ret_obj = m_pLast->Get();
      }
      return ret_obj;
    }

    void Clear()
    {
      if ( m_pLast )
      {
        m_pLast->Clear();
        m_pLast = m_pFirst;
      }
    }

  private:

    T_CHUNK *m_pFirst, *m_pLast;
  };

}