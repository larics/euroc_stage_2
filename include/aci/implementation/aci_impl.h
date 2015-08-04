/*
* Copyright (c) 2014, Markus Achtelik, ASL, ETH Zurich, Switzerland
* You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#ifndef ACI_IMPL_H_
#define ACI_IMPL_H_

#include <memory>
#include <map>
#include <vector>
#include <chrono>

#include <aci/aci.h>
#include <aci/macros.h>

namespace aci {

template<class VariableType_>
Variable<VariableType_> Aci::registerVariable(int packet_id, int variable_id) {

  ACI_MEM_TABLE_ENTRY* aci_variable = aciGetVariableItemById(variable_id);
  if(aci_variable == NULL){
    ACI_WARN_STREAM("Variable with id 0x" << std::hex << variable_id << " is not available on the device");
    return Variable<VariableType_>();
  }

  if (aci_variable->varType != Variable<VariableType_>::AsctecVariableType) {
    ACI_ERROR_STREAM("Wanted to register variable 0x" << std::hex << variable_id << " with type 0x" <<
                     Variable<VariableType_>::AsctecVariableType << " but device reported type 0x" <<
                     static_cast<int>(aci_variable->varType));
    return Variable<VariableType_>();
  }

  VariablePacket& packet = variable_packets_.at(packet_id);
  VariablePacket::const_iterator it = packet.find(variable_id);

  // Create a new variable, if we don't have it yet.
  if (it == packet.end()) {
    std::shared_ptr<Variable<VariableType_> > var(new Variable<VariableType_>(variable_id));
    void* var_data_ptr = var->getRawPtr();
    packet.insert(std::make_pair(variable_id, var));
    aciAddContentToVarPacket(packet_id, variable_id, var_data_ptr);
    return *var;
  }
  else {
    std::shared_ptr<Variable<VariableType_> > var =
        std::dynamic_pointer_cast<Variable<VariableType_> >(it->second);
    if (!var) {
      ACI_ERROR_STREAM("Registering variable 0x" << std::hex << variable_id << " with type 0x" <<
                       Variable<VariableType_>::AsctecVariableType <<
                       " failed.The same variable is already registered as type 0x" << it->second->getType());
      std::exit(EXIT_FAILURE);  // Rude, but safer to do it here, than having an error later.
    }
    return *var;
  }
}

template<class VariableType_>
Variable<VariableType_> Aci::registerCommand(int packet_id, int command_id) {

  ACI_MEM_TABLE_ENTRY* aci_command = aciGetCommandItemById(command_id);
  if(aci_command == NULL){
    ACI_WARN_STREAM("Command with id " << command_id << " is not available on the device");
    return Variable<VariableType_>();
  }

  if (aci_command->varType != Variable<VariableType_>::AsctecVariableType) {
    ACI_ERROR_STREAM("Wanted to register command 0x" << std::hex << command_id << " with type 0x" <<
                     Variable<VariableType_>::AsctecVariableType << " but device reported type 0x"
                     << static_cast<int>(aci_command->varType));
    return Variable<VariableType_>();
  }

  VariablePacket& packet = command_packets_.at(packet_id);
  VariablePacket::const_iterator it = packet.find(command_id);

  free_command_packet_ids_.remove(packet_id);

  // Create a new variable, if we don't have it yet.
  if (it == packet.end()) {
    std::shared_ptr<Variable<VariableType_> > var(new Variable<VariableType_>(command_id));
    void* var_data_ptr = var->getRawPtr();
    packet.insert(std::make_pair(command_id, var));
    aciAddContentToCmdPacket(packet_id, command_id, var_data_ptr);
    return *var;
  }
  else {
    ACI_WARN_STREAM("The command with ID " << command_id << " was already registered for this packet.");
    std::shared_ptr<Variable<VariableType_> > var =
        std::dynamic_pointer_cast<Variable<VariableType_> >(it->second);
    if (!var) {
      ACI_ERROR_STREAM("Registering command 0x" << std::hex << command_id << " with type 0x" <<
                       Variable<VariableType_>::AsctecVariableType <<
                       " failed. The same command is already registered as type 0x" << it->second->getType());
      std::exit(EXIT_FAILURE);  // Rude, but safer to do it here, than having an error later.
    }
    return *var;
  }
}

template<class VariableType_, class Representation, class Duration>
bool Aci::requestSingleVariable(Variable<VariableType_>& variable, int id,
                           const std::chrono::duration<Representation, Duration>& timeout) {

  UniqueLock lock(mutex_);
  aciRequestSingleVariable(id);
  auto time_now = std::chrono::system_clock::now();

  bool ret =  single_request_condition_.wait_until(lock, time_now + timeout,[this, id] {return id == single_request_returned_id_;});

  single_request_returned_id_ = 0;

  if (!ret)
    return false;

  memcpy(variable.getRawPtr(), single_request_buffer_, Variable<VariableType_>::Size);

  return true;
}

} // end namespace aci

#endif /* ACI_H_ */
