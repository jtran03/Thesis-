// // Importing Modules ---------------------------------------------------------
// console.log(`##### Importing Modules Tutorial #####`)
// const tutorial = require('./tutorial')
// console.log(tutorial)
// console.log(`\n`)

// // Event Driven Code ---------------------------------------------------------
// console.log(`##### Event Driven Tutorial #####`)
// const EventEmitter = require('events');
// const eventEmitter = new EventEmitter(); 

// // Code that runs when an emitter is called 
// eventEmitter.on('tutorial',(num1, num2)=>{
//     console.log('Tutorial event has occured'); 
//     console.log(num1 + num2); 
// }); 

// // Call an emit 
// eventEmitter.emit('tutorial', 1,2)

// // Extend emitters to a class 
// class Person extends EventEmitter{
    
//     // Default constructor with argument 'name' 
//     constructor(name){
//         super();
//         // Create a name variable 
//         this._name = name; 
//     }

//     // Getter (returns the name)
//     get name(){
//         return this._name; 
//     }
// }

// // Create a new person class that can emit 
// let pedro = new Person('Pedro'); 
// let christina = new Person('Christina'); 


// // Listener function. It is listening for 'name' 
// pedro.on('name',()=>{
//     console.log('my name is ' + pedro.name);
// })

// christina.on('name',()=>{
//     console.log('my name is ' + christina.name);
// })


// // Call an emit. must include 'name' since the listener is looking for 'name'
// pedro.emit('name')
// christina.emit('name')

// // Readline module ---------------------------------------------------------
// console.log(`##### Readline module #####`)
// const readline = require('readline');

// // Create a readline interface
// const rl = readline.createInterface({input : process.stdin, 
//                                     output : process.stdout});

// // Generate two random numbers 
// let num1 = Math.floor((Math.random() * 10 ) + 1); 
// let num2 = Math.floor((Math.random() * 10 ) + 1); 
// let answer = num1 + num2; 

// // Ask the user as a question 
// rl.question(`What is ${ num1 } + ${ num2 }? \n`, (userInput)=>{
    
//     // Check if user input is correct. trim() removes trailing whitespace
//     if(userInput.trim() == answer){

//         // Close the readline module --> this also emits a close event 
//         rl.close(); 

//     }

//     else{

//         // Create a prompt
//         rl.setPrompt('Incorrect response please try again\n');

//         // Re-prompt the user
//         rl.prompt();

//         // Listen to the readline interface
//         rl.on('line',(userInput)=>{

//             if(userInput.trim() == answer)
                
//                 // close the readline module 
//                 rl.close();

//             else{

//                 // Create a new prompt 
//                 rl.setPrompt(`Your answer of ${ userInput } is incorrect\n`)

//                 // Prompt the user
//                 rl.prompt(); 
//             } 
//         })
//     }
//     }
// );

// // This listens for a close event. ()=> is a cl 
// rl.on('close',()=>{
//     console.log('Correct!!!!'); 
// })

// //File System Module ---------------------------------------------------------
// console.log(`##### File System Module #####`)

// // Create a file system module 
// const fs = require('fs')

// // Create a file 
// fs.writeFile(`example.txt`, "this is an example", (err)=>{

//     // Check for error 
//     if (err)

//         console.log(err);

//     else 

//         console.log(`File successfully created)`)

//         // Read the file in utf8 format  
//         fs.readFile(`example.txt`, 'utf8', (err,file)=>{
        
//             if(err)

//                 console.log(err);

//             else 

//                 console.log(file);
//         })
// }); 

// // Rename the file [target][new name][error]
// fs.rename('example.txt', 'example2.txt', (err)=>{

//     if (err)

//         console.log(err);

//     else   

//         console.log('successfully renamed the file');

// });

// // Appending data to a file 
// fs.appendFile('example2.txt', "Some data being appended", (err)=>{

//     if(err)

//         console.log(err);

//     else 

//         console.log('Successfully appended data to file')

// }); 

// // Delete the file 
// fs.unlink('example2.txt',(err)=>{

//     if(err)

//         console.log(err);

//     else 

//         console.log('Successfully deleted file')

// })

// // Create a new directory 
// fs.mkdir('tutorial',(err)=>{
//     if(err)
//         console.log(err);
//     else{
//         console.log('folder successfully created')
//     }
// })

// // Delete a directory 
// fs.rmdir('tutorial'),(err)=>{
//     if(err)
//         console.log(err);
//     else{
//         console.log('successfully deleted the folder')
//     }
// }

// // Read/Write Streams 
// const fs = require('fs');

// // This is to manipualate an input file 
// const zlib = require('zlib');
// const gzip = zlib.createGzip(); 
// // Unzip a file 
// // const gzip = zlib.createGunzip(); 

// const readStream = fs.createReadStream(`./example.txt`, 'utf-8')
// const writeStream = fs.createWriteStream('./example2.txt.gz'); 

// // Listen to the data 
// readStream.on('data',(chunk)=>{
//     writeStream.write(chunk);
// })

// // Does the same thing ^ above 
// readStream.pipe(writeStream); 

// // Also compresses the data 
// readStream.pipe(gzip).pipe(writeStream); 

// HTTP Module ------------------------------------------------------------------
// install the http module 
// const http = require('http'); 

// // Call amethod called 'createServer' which will create a http server object 
// // Will return a request and response callback
// const server = http.createServer((req,res)=>{

//     // This is a request (example: check if we are in host domain)
//     if (req.url === '/'){

//         // This is a response 
//         res.write('Hello world from nodejs');

//         // Send the response
//         res.end(); 

//     }
//     else{

//         // Response 
//         res.write('using some other domain')
//         res.end();
//     }

// }); 

// // Listen for 3000  
// server.listen('3000');

// Seving HTML File ------------------------------------------------------------------
const http = require('http');
const fs = require('fs)');
http.createServer((req,res)=>{

    // Create a read stream //switch with .png //.json 
    const readStream = fs.createReadStream('./static/index.html')

    // Write a header (let the client know what type of data im expecting)
    // Status code: 200 is good status --> http status codes switch with png//json
    res.writeHead(200,{'Content-type': 'text/html'});

    // Pipe html to the response  
    readStream.pipe(res);
    
}).listen(3000)

// Express js File ------------------------------------------------------------------


