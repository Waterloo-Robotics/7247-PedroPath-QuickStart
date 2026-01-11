package org.firstinspires.ftc.teamcode.Modules;

public class Table2D
{
    private float tableX[];
    private float tableZ[];

    private boolean descendingX;

    public Table2D(float[] tableX, float[] tableZ) {
        this.tableX = tableX;
        this.tableZ = tableZ;

        this.descendingX = false;
    }

    public float Lookup(float X)
    {
        /* Locals */
        float lowerX = 0;
        int lowerXIndex = 0;
        float upperX = 0;
        int upperXIndex = 0;

        float lowerX_value;
        float upperX_value;

        float output = 0;

        /* Constants */


        /* Check if X is at or below the first x value in the table */
        if (this.descendingX)
        {
            if (X >= this.tableX[0])
            {
                lowerX = this.tableX[0];
                upperX = this.tableX[1];

                lowerXIndex = 0;
                upperXIndex = 1;
            }
            /* Check if X if at or above the last x value in the table */
            else if (X <= this.tableX[tableX.length - 1])
            {
                lowerX = this.tableX[tableX.length - 1];
                upperX = this.tableX[tableX.length - 2];

                lowerXIndex = tableX.length - 1;
                upperXIndex = tableX.length - 2;
            }
            /* X is somewhere in the middle, interpolate between two values */
            else
            {
                /* Find upper and lower X values */
                for (int i=0; i<this.tableX.length; i++)
                {
                    if (this.tableX[i] < X)
                    {
                        /* We're now above table x value, set the lower and upper limits then break */
                        lowerX = this.tableX[i];
                        upperX = this.tableX[i-1];

                        lowerXIndex = i;
                        upperXIndex = i-1;
                        break;
                    }
                }
            }
        }
        /* X is ascending */
        else
        {
            if (X <= this.tableX[0])
            {
                lowerX = this.tableX[0];
                upperX = this.tableX[1];

                lowerXIndex = 0;
                upperXIndex = 1;
            }
            /* Check if X if at or above the last x value in the table */
            else if (X >= this.tableX[tableX.length - 1])
            {
                lowerX = this.tableX[tableX.length - 2];
                upperX = this.tableX[tableX.length - 1];

                lowerXIndex = tableX.length - 2;
                upperXIndex = tableX.length - 1;
            }
            /* X is somewhere in the middle, interpolate between two values */
            else
            {
                /* Find upper and lower X values */
                for (int i=0; i<this.tableX.length; i++)
                {
                    if (this.tableX[i] > X)
                    {
                        /* We're now above table x value, set the lower and upper limits then break */
                        lowerX = this.tableX[i-1];
                        upperX = this.tableX[i];

                        lowerXIndex = i - 1;
                        upperXIndex = i;
                        break;
                    }
                }
            }
        }

        lowerX_value = this.tableZ[lowerXIndex];
        upperX_value = this.tableZ[upperXIndex];

        output = lowerX_value * ((upperX - X) / (upperX - lowerX)) +
                upperX_value * ((X - lowerX) / (upperX - lowerX));

        return output;
    }

    public void setDescendingX(boolean descendingX) {
        this.descendingX = descendingX;
    }

}