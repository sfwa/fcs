/*
Use EDMA for EMIFs, and run without FIFOs. Use interrupts for read, and rate
limit in software for writes (1 byte per 20us or so).
*/
