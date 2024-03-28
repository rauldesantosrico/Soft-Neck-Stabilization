#ifndef LINEAR_SYSTEM
#define LINEAR_SYSTEM
class LinearSystem
{
public:
    LinearSystem();

    void printSystem();
    //operator overloading
    friend double operator > (double input, LinearSystem& output)
    {
        return output.output(input);
    }

    friend double operator > (LinearSystem& from, double & to)
    {
        to=from.GetCurrentOut();
        return to;
    }

    long SetSaturation(double low, double high);



protected:
    double maxOut, minOut; // 0 values disables saturation
    bool saturation;


private:
    virtual double GetCurrentOut() const = 0;
    virtual double output(double new_input)=0;
};

#endif
