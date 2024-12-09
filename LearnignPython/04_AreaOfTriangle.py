#This is program to find the area of the Valid Triangle

a=int(input("Enter the first side of Triangle A: "));
b=int(input("Enter the Second side of Triangle B:"));
c=int(input("Enter the Third side of Triangle C:"));

#Logic to calculate the Area of triangle
area=(a+b+c)/2;

#Displaying the Result
print("The area of A={0} B={1} and C {2} is = {3}".format(a,b,c,area))