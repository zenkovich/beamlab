#pragma once

#include <vector>

namespace Perfmon
{
    // Circular buffer for storing time series data with statistical functions
    template<typename T>
    class TimeSeries
    {
    public:
        // Constructor with fixed number of samples
        explicit TimeSeries(size_t numSamples)
            : _head(0), _numSamples(numSamples), _dirty(true)
        {
            _values.resize(_numSamples, (T)0);
            _sortedValues.resize(_numSamples, (T)0);
        }

        // Add new value to circular buffer, returns overwritten value
        T Push(T v) noexcept
        {
            const T lrv = _values[_head];

            _values[_head++] = v;
            if (_head >= _numSamples)
                _head = 0;

            Invalidate();

            return lrv;
        }

        // Get total number of samples in buffer
        size_t GetNumSamples() const noexcept
        {
            return _numSamples;
        }

        // Get pointer to raw samples array
        const T* GetSamples() const noexcept
        {
            return _values.data();
        }

        // Get current head position in circular buffer
        size_t GetHead() const noexcept
        {
            return _head;
        }

        // Get most recently added sample
        T GetMostRecentSample() const noexcept
        {
            return _values[(_head + _numSamples - 1) % _numSamples];
        }

        // Get minimum value (alias for P0)
        T Min() const noexcept
        {
            return P0();
        }

        // Get maximum value (alias for P100)
        T Max() const noexcept
        {
            return P100();
        }

        // Get 0th percentile (minimum)
        T P0() const noexcept
        {
            Validate();
            return _sortedValues[0];
        }

        // Get 25th percentile
        T P25() const noexcept
        {
            Validate();
            return _sortedValues[_numSamples / 4];
        }

        // Get 50th percentile (median)
        T P50() const noexcept
        {
            Validate();
            return _sortedValues[_numSamples / 2];
        }

        // Get 75th percentile
        T P75() const noexcept
        {
            Validate();
            return _sortedValues[3 * _numSamples / 4];
        }

        // Get 100th percentile (maximum)
        T P100() const noexcept
        {
            Validate();
            return _sortedValues[_numSamples - 1];
        }

    private:
        // Sort values for percentile calculations if needed
        void Validate() const noexcept
        {
            if (!_dirty)
                return;

            std::copy(_values.begin(), _values.end(), _sortedValues.begin());
            std::sort(_sortedValues.begin(), _sortedValues.end());

            _dirty = false;
        }

        // Mark sorted values as invalid
        void Invalidate() noexcept
        {
            _dirty = true;
        }

        std::vector<T> _values;              // Circular buffer for values
        mutable std::vector<T> _sortedValues; // Sorted copy for percentiles
        size_t _head;                        // Current position in circular buffer
        size_t _numSamples;                  // Total buffer size
        mutable bool _dirty;                 // Whether sorted values need update
    };
}
