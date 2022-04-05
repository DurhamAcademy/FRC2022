package frc.kyberlib.tutorial.Tutorial1_Kotlin

// this is a comment. It lets me type things without actually telling the computer to do anything
// I make a comment my putting 2 '/' at the beginning of the line

// variables and types
fun lesson1() {
    var variable = 1  // this is a variable. It stores a value so that you can use it by typing the variables name
    variable = 2 // by using '=' I can change the value that the name "variable" represents (I don't need far because I've already named this variable)

    println("This is a ")
    println(variable)  // println(variable) types out the value whatever is in the parenthesises
    println(variable+variable)

    // some other things you can store
    var number = 1
    var preciseNumber = 1.234  // names should not have spaces so weTypeLikeThis
    var words = "This is a bunch of letters"
//    var invalid = this is a bunch of letters // this code doesn't work because the right isn't in quotations. It is trying to read it as code
    var correct = true  // this represents a true statement
    var incorrect = false

    println(number)
    println(preciseNumber)
    println(words)
    println(correct)
    println(incorrect)
}

// logic
fun lesson2() {
    if(true) {
        println("This runs")
    }

    if(false) {
        println("this won't run")
    }

    if(false) {
        println("this won't run")
    } else {
        println("this outputs because the previous didn't")
    }

    if (1<2) {
        println("1 is indeed less than 2")
    } else {
        println("since the previous block happened, this won't")
    }
}



fun main() {  // ignore this for now
}