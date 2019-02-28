# Hawk Path
Hawk Path, an external Java webapp, is used for fast and simple path creation. The app allows a user to create and visualize autonomous paths.

## Getting Started
These instructions will get you a copy of the project up and running on your local machine for development, testing, and use.

### Dependencies
- spring-webmvc 4.3.0.RELEASE
- javax.servlet-api 3.0.1
- jstl 1.2
- jama 1.0.3

### Installing
- Clone this Repository 
- [Download](https://maven.apache.org/download.cgi)  and [Install](https://maven.apache.org/install.html) Maven or use Homebrew (`brew install maven`)
- Run App (`mvn tomcat7:run`)
- Open [http://localhost:8080/](http://localhost:8080/)

### Import Into IDEs
- To import into the Eclipse IDE, run `mvn eclipse:eclipse`, then open project in Eclipse
- Open IntelliJ, select Open, click on the `pom.xml` file, and select Open as Project