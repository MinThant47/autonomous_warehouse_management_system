import axios from "axios";

// Use the same computer that served the frontend, rather than localhost,
// so phones/tablets on the LAN call the backend on the development machine.
const backendHost = window.location.hostname || "localhost";
const API = axios.create({
    baseURL: `http://${backendHost}:8000`
});


export default API;
