# ESP32 Motion Monitoring System

## Overview

A real-time IoT motion monitoring prototype built using an ESP32 and MPU6050.

The system classifies movement into:

- Stable
- Motion
- Shake

The ESP32 provides:

- LED status indication
- Vibration feedback
- Wi-Fi connectivity
- Cloud logging via Google Apps Script
- Google Sheets data storage
- Live Looker Studio dashboard

---

## Hardware

- ESP32 Dev Module
- MPU6050
- WS2812B NeoPixel
- Vibration Motor
- Push Button

---

## Software

- Arduino IDE
- Google Apps Script
- Google Sheets
- Looker Studio

---

## Features

- Motion classification
- LED feedback
- Haptic feedback
- Wi-Fi upload
- Cloud logging
- Dashboard visualization

---

## Project Architecture

MPU6050
↓
ESP32
↓
Motion Classification
↓
LED + Vibration
↓
Wi-Fi
↓
Google Apps Script
↓
Google Sheets
↓
Looker Studio
