import React from 'react'
import ReactDOM from 'react-dom/client'
import { BrowserRouter } from 'react-router-dom'
import App from './App.jsx'
import { BotProvider } from './lib/store.jsx'
import './styles.css'

ReactDOM.createRoot(document.getElementById('root')).render(
  <React.StrictMode>
    <BrowserRouter>
      <BotProvider>
        <App />
      </BotProvider>
    </BrowserRouter>
  </React.StrictMode>,
)
