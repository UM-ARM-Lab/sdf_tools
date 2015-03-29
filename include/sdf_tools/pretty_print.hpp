#include <iostream>
#include <vector>
#include <list>
#include <map>
#include <unordered_map>
#include <string>

#ifndef PRETTY_PRINT_HPP
#define PRETTY_PRINT_HPP

// Handy functions for printing vectors and pairs
template <typename T>
std::string PrettyPrint(const T& toprint, const bool add_delimiters=false)
{
    std::ostringstream strm;
    strm << toprint;
    return strm.str();
}

template <typename T>
std::string PrettyPrint(const std::vector<T>& vectoprint, const bool add_delimiters=false)
{
    std::ostringstream strm;
    if (vectoprint.size() > 0)
    {
        if (add_delimiters)
        {
            strm << "[" << PrettyPrint(vectoprint[0], add_delimiters);
            for (size_t idx = 1; idx < vectoprint.size(); idx++)
            {
                strm << ", " << PrettyPrint(vectoprint[idx], add_delimiters);
            }
            strm << "]";
        }
        else
        {
            strm << PrettyPrint(vectoprint[0], add_delimiters);
            for (size_t idx = 1; idx < vectoprint.size(); idx++)
            {
                strm << ", " << PrettyPrint(vectoprint[idx], add_delimiters);
            }
        }
    }
    return strm.str();
}

template <typename T>
std::string PrettyPrint(const std::list<T>& listtoprint, const bool add_delimiters=false)
{
    std::ostringstream strm;
    if (listtoprint.size() > 0)
    {
        if (add_delimiters)
        {
            strm << "[";
            typename std::list<T>::const_iterator itr;
            for (itr = listtoprint.begin(); itr != listtoprint.end(); ++itr)
            {
                if (itr != listtoprint.begin())
                {
                    strm << ", " << PrettyPrint(*itr, add_delimiters);
                }
                else
                {
                    strm << PrettyPrint(*itr, add_delimiters);
                }
            }
            strm << "]";
        }
        else
        {
            typename std::list<T>::const_iterator itr;
            for (itr = listtoprint.begin(); itr != listtoprint.end(); ++itr)
            {
                if (itr != listtoprint.begin())
                {
                    strm << ", " << PrettyPrint(*itr, add_delimiters);
                }
                else
                {
                    strm << PrettyPrint(*itr, add_delimiters);
                }
            }
        }
    }
    return strm.str();
}


template <typename A, typename B>
std::string PrettyPrint(const std::pair<A, B>& pairtoprint, const bool add_delimiters=false)
{
    std::ostringstream strm;
    if (add_delimiters)
    {
        strm << "<" << PrettyPrint(pairtoprint.first, add_delimiters) << ": " << PrettyPrint(pairtoprint.second, add_delimiters) << ">";
    }
    else
    {
        strm << PrettyPrint(pairtoprint.first, add_delimiters) << ": " << PrettyPrint(pairtoprint.second, add_delimiters);
    }
    return strm.str();
}

template <typename A, typename B>
std::string PrettyPrint(const std::map<A, B>& maptoprint, const bool add_delimiters=false)
{
    std::ostringstream strm;
    if (maptoprint.size() > 0)
    {
        if (add_delimiters)
        {
            strm << "{";
            typename std::map<A, B>::const_iterator itr;
            for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
            {
                std::pair<A, B> cur_pair(itr->first, itr->second);
                if (itr != maptoprint.begin())
                {
                    strm << ", " << PrettyPrint(cur_pair, add_delimiters);
                }
                else
                {
                    strm << PrettyPrint(cur_pair, add_delimiters);
                }
            }
            strm << "}";
        }
        else
        {
            typename std::map<A, B>::const_iterator itr;
            for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
            {
                std::pair<A, B> cur_pair(itr->first, itr->second);
                if (itr != maptoprint.begin())
                {
                    strm << ", " << PrettyPrint(cur_pair, add_delimiters);
                }
                else
                {
                    strm << PrettyPrint(cur_pair, add_delimiters);
                }
            }
        }
    }
    return strm.str();
}

template <typename A, typename B>
std::string PrettyPrint(const std::unordered_map<A, B>& maptoprint, const bool add_delimiters=false)
{
    std::ostringstream strm;
    if (maptoprint.size() > 0)
    {
        if (add_delimiters)
        {
            strm << "{";
            typename std::unordered_map<A, B>::const_iterator itr;
            for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
            {
                std::pair<A, B> cur_pair(itr->first, itr->second);
                if (itr != maptoprint.begin())
                {
                    strm << ", " << PrettyPrint(cur_pair, add_delimiters);
                }
                else
                {
                    strm << PrettyPrint(cur_pair, add_delimiters);
                }
            }
            strm << "}";
        }
        else
        {
            typename std::unordered_map<A, B>::const_iterator itr;
            for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
            {
                std::pair<A, B> cur_pair(itr->first, itr->second);
                if (itr != maptoprint.begin())
                {
                    strm << ", " << PrettyPrint(cur_pair, add_delimiters);
                }
                else
                {
                    strm << PrettyPrint(cur_pair, add_delimiters);
                }
            }
        }
    }
    return strm.str();
}

#endif // PRETTY_PRINT_HPP
