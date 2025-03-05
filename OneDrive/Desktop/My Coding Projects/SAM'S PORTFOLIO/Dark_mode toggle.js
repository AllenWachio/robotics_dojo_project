// Select the button element
const themeToggleButton = document.getElementById("theme-toggle");

// Check for saved user preference in localStorage
document.addEventListener("DOMContentLoaded", () => {
    const darkModeEnabled = localStorage.getItem("darkMode") === "true";
    if (darkModeEnabled) {
        document.body.classList.add("dark-mode");
        themeToggleButton.textContent = "☀️ Light Mode"; // Update button text
    }
});

// Add event listener for the toggle button
themeToggleButton.addEventListener("click", () => {
    // Toggle the "dark-mode" class on the <body>
    document.body.classList.toggle("dark-mode");

    // Update the button text
    const isDarkMode = document.body.classList.contains("dark-mode");
    themeToggleButton.textContent = isDarkMode ? "☀️ Light Mode" : "🌙 Dark Mode";

    // Save the user's preference in localStorage
    localStorage.setItem("darkMode", isDarkMode);
});
const toggleButton = document.getElementById('theme-toggle');
toggleButton.addEventListener('click', () => {
    const currentTheme = document.body.getAttribute('data-theme');
    const newTheme = currentTheme === 'dark' ? 'light' : 'dark';
    document.body.setAttribute('data-theme', newTheme);
});
document.body.setAttribute('data-theme', 'dark'); // To test dark mode manually