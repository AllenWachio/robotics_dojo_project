

### **1. What is black-box testing, and how does it differ from white-box testing?**  
**Black-box testing** is a software testing technique that evaluates the functionality of a system based on its inputs and expected outputs without knowing the internal structure or implementation details.  

**White-box testing**, on the other hand, involves testing the internal logic, structure, and code paths of the application. Testers need programming knowledge to examine conditions, loops, and branches in the source code.

**Key Differences:**
| Feature | Black-Box Testing | White-Box Testing |
|---------|------------------|------------------|
| Focus | Functionality | Internal code structure |
| Tester Knowledge | No knowledge of internal code required | Requires programming knowledge |
| Techniques Used | Equivalence Partitioning, Boundary Value Analysis, Decision Tables | Statement Coverage, Branch Coverage, Condition Coverage |
| Examples | UI Testing, System Testing, Acceptance Testing | Unit Testing, Code Reviews, Path Testing |

---

### **2. Explain the concept of equivalence partitioning and how it helps in reducing test cases.**  
**Equivalence Partitioning (EP)** is a **black-box testing** technique used to divide input data into groups (partitions) that are expected to produce similar behavior. Instead of testing every possible input, a tester selects one representative value from each partition, reducing the number of test cases while maintaining coverage.

#### **Example:**  
Suppose a system accepts numbers from **1 to 100** as valid inputs. Using equivalence partitioning, we can divide the input into three groups:  
1. **Valid partition:** 1–100 (e.g., test with **50**)  
2. **Invalid partition (too low):** Less than 1 (e.g., test with **-5**)  
3. **Invalid partition (too high):** Greater than 100 (e.g., test with **150**)  

This method **reduces redundancy** while ensuring meaningful test coverage.

---

### **3. What is boundary value analysis, and why is it useful in software testing?**  
**Boundary Value Analysis (BVA)** is a testing technique that focuses on the boundaries of input ranges since defects are more likely to occur at these points.

#### **Why is it useful?**  
- Identifies edge cases that may cause errors.  
- Finds defects at the limits of input conditions.  
- Reduces the number of test cases compared to exhaustive testing.  

#### **Example:**  
If a system accepts ages **between 18 and 60**, the test cases would be:  
- **Lower boundary:** 17 (invalid), **18 (valid)**  
- **Upper boundary:** **60 (valid)**, 61 (invalid)  

BVA helps uncover bugs related to **off-by-one errors** (e.g., using `> 18` instead of `>= 18` in code).

---

### **4. Describe the purpose of decision tables in black-box testing and provide an example scenario where they might be used.**  
A **decision table** is a structured way of representing logical conditions and their corresponding actions. It is used in black-box testing when multiple conditions affect an outcome.

#### **Purpose:**  
- Helps design test cases for complex business rules.  
- Ensures all decision combinations are tested.  
- Improves clarity in handling multiple conditions.

#### **Example:** Bank Loan Approval System  
A bank grants loans based on two conditions:  
1. **Credit Score ≥ 700**  
2. **Stable Income**  

| Credit Score ≥ 700 | Stable Income | Loan Approved? |
|-------------------|--------------|---------------|
| Yes | Yes | ✅ Yes |
| Yes | No | ❌ No |
| No | Yes | ❌ No |
| No | No | ❌ No |

By creating a decision table, testers can ensure that all rule combinations are covered.

---

### **5. What is state transition testing, and how can it be applied to a user account system?**  
**State Transition Testing** is a technique used to test how a system transitions from one state to another based on user actions or inputs.

#### **Application to a User Account System:**  
A user account can have different states, such as **Active, Locked, and Disabled**. The system transitions between these states based on actions like login attempts.

| Current State | Action | New State |
|--------------|--------|-----------|
| Active | 3 Failed Login Attempts | Locked |
| Locked | Password Reset | Active |
| Active | Deactivation | Disabled |
| Disabled | Admin Reactivation | Active |

By testing these transitions, testers ensure the system handles state changes correctly.

