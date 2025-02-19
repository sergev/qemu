/* AUTOMATICALLY GENERATED, DO NOT MODIFY */

/*
 * schema-defined QAPI visitor functions
 *
 * Copyright IBM, Corp. 2011
 *
 * Authors:
 *  Anthony Liguori   <aliguori@us.ibm.com>
 *
 * This work is licensed under the terms of the GNU LGPL, version 2.1 or later.
 * See the COPYING.LIB file in the top-level directory.
 *
 */

#ifndef QGA_QAPI_VISIT_H
#define QGA_QAPI_VISIT_H

#include "qapi/visitor.h"
#include "qga-qapi-types.h"


#ifndef QAPI_VISIT_BUILTIN_VISITOR_DECL_H
#define QAPI_VISIT_BUILTIN_VISITOR_DECL_H

void visit_type_int32List(Visitor *m, int32List **obj, const char *name, Error **errp);
void visit_type_int16List(Visitor *m, int16List **obj, const char *name, Error **errp);
void visit_type_intList(Visitor *m, intList **obj, const char *name, Error **errp);
void visit_type_uint8List(Visitor *m, uint8List **obj, const char *name, Error **errp);
void visit_type_numberList(Visitor *m, numberList **obj, const char *name, Error **errp);
void visit_type_int8List(Visitor *m, int8List **obj, const char *name, Error **errp);
void visit_type_sizeList(Visitor *m, sizeList **obj, const char *name, Error **errp);
void visit_type_uint64List(Visitor *m, uint64List **obj, const char *name, Error **errp);
void visit_type_uint16List(Visitor *m, uint16List **obj, const char *name, Error **errp);
void visit_type_boolList(Visitor *m, boolList **obj, const char *name, Error **errp);
void visit_type_strList(Visitor *m, strList **obj, const char *name, Error **errp);
void visit_type_int64List(Visitor *m, int64List **obj, const char *name, Error **errp);
void visit_type_uint32List(Visitor *m, uint32List **obj, const char *name, Error **errp);

#endif /* QAPI_VISIT_BUILTIN_VISITOR_DECL_H */


void visit_type_GuestAgentCommandInfo(Visitor *m, GuestAgentCommandInfo **obj, const char *name, Error **errp);
void visit_type_GuestAgentCommandInfoList(Visitor *m, GuestAgentCommandInfoList **obj, const char *name, Error **errp);

void visit_type_GuestAgentInfo(Visitor *m, GuestAgentInfo **obj, const char *name, Error **errp);
void visit_type_GuestAgentInfoList(Visitor *m, GuestAgentInfoList **obj, const char *name, Error **errp);

void visit_type_GuestFileRead(Visitor *m, GuestFileRead **obj, const char *name, Error **errp);
void visit_type_GuestFileReadList(Visitor *m, GuestFileReadList **obj, const char *name, Error **errp);

void visit_type_GuestFileWrite(Visitor *m, GuestFileWrite **obj, const char *name, Error **errp);
void visit_type_GuestFileWriteList(Visitor *m, GuestFileWriteList **obj, const char *name, Error **errp);

void visit_type_GuestFileSeek(Visitor *m, GuestFileSeek **obj, const char *name, Error **errp);
void visit_type_GuestFileSeekList(Visitor *m, GuestFileSeekList **obj, const char *name, Error **errp);

void visit_type_GuestFsfreezeStatus(Visitor *m, GuestFsfreezeStatus *obj, const char *name, Error **errp);
void visit_type_GuestFsfreezeStatusList(Visitor *m, GuestFsfreezeStatusList **obj, const char *name, Error **errp);

void visit_type_GuestIpAddressType(Visitor *m, GuestIpAddressType *obj, const char *name, Error **errp);
void visit_type_GuestIpAddressTypeList(Visitor *m, GuestIpAddressTypeList **obj, const char *name, Error **errp);

void visit_type_GuestIpAddress(Visitor *m, GuestIpAddress **obj, const char *name, Error **errp);
void visit_type_GuestIpAddressList(Visitor *m, GuestIpAddressList **obj, const char *name, Error **errp);

void visit_type_GuestNetworkInterface(Visitor *m, GuestNetworkInterface **obj, const char *name, Error **errp);
void visit_type_GuestNetworkInterfaceList(Visitor *m, GuestNetworkInterfaceList **obj, const char *name, Error **errp);

void visit_type_GuestLogicalProcessor(Visitor *m, GuestLogicalProcessor **obj, const char *name, Error **errp);
void visit_type_GuestLogicalProcessorList(Visitor *m, GuestLogicalProcessorList **obj, const char *name, Error **errp);

void visit_type_GuestDiskBusType(Visitor *m, GuestDiskBusType *obj, const char *name, Error **errp);
void visit_type_GuestDiskBusTypeList(Visitor *m, GuestDiskBusTypeList **obj, const char *name, Error **errp);

void visit_type_GuestPCIAddress(Visitor *m, GuestPCIAddress **obj, const char *name, Error **errp);
void visit_type_GuestPCIAddressList(Visitor *m, GuestPCIAddressList **obj, const char *name, Error **errp);

void visit_type_GuestDiskAddress(Visitor *m, GuestDiskAddress **obj, const char *name, Error **errp);
void visit_type_GuestDiskAddressList(Visitor *m, GuestDiskAddressList **obj, const char *name, Error **errp);

void visit_type_GuestFilesystemInfo(Visitor *m, GuestFilesystemInfo **obj, const char *name, Error **errp);
void visit_type_GuestFilesystemInfoList(Visitor *m, GuestFilesystemInfoList **obj, const char *name, Error **errp);

void visit_type_GuestMemoryBlock(Visitor *m, GuestMemoryBlock **obj, const char *name, Error **errp);
void visit_type_GuestMemoryBlockList(Visitor *m, GuestMemoryBlockList **obj, const char *name, Error **errp);

void visit_type_GuestMemoryBlockResponseType(Visitor *m, GuestMemoryBlockResponseType *obj, const char *name, Error **errp);
void visit_type_GuestMemoryBlockResponseTypeList(Visitor *m, GuestMemoryBlockResponseTypeList **obj, const char *name, Error **errp);

void visit_type_GuestMemoryBlockResponse(Visitor *m, GuestMemoryBlockResponse **obj, const char *name, Error **errp);
void visit_type_GuestMemoryBlockResponseList(Visitor *m, GuestMemoryBlockResponseList **obj, const char *name, Error **errp);

void visit_type_GuestMemoryBlockInfo(Visitor *m, GuestMemoryBlockInfo **obj, const char *name, Error **errp);
void visit_type_GuestMemoryBlockInfoList(Visitor *m, GuestMemoryBlockInfoList **obj, const char *name, Error **errp);

#endif
